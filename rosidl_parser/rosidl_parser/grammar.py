# Copyright 2018 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import pathlib

from lark import Lark
from lark.lexer import Token
from lark.reconstruct import Reconstructor
from lark.tree import Tree

from rosidl_parser import Field
from rosidl_parser import MessageSpecification
from rosidl_parser import ServiceSpecification
from rosidl_parser import Type

with open(os.path.join(os.path.dirname(__file__), 'grammar.lark'), 'r') as h:
    grammar = h.read()

parser = Lark(grammar, start='specification')
reconstructor = Reconstructor(parser)


def parse_idl_file(package_name, namespace, file_):
    with open(file_, 'r') as h:
        content = h.read()
    try:
        return parse_idl_string(
            package_name, namespace, pathlib.Path(file_).stem, content)
    except Exception as e:
        print(file_)
        raise


def parse_idl_string(package_name, namespace, interface_name, idl_string):
    global parser
    tree = parser.parse(idl_string)
    c = count(tree, 'struct_def')

    if c == 1:
        msg = MessageSpecification(package_name, namespace, interface_name, [], [])
        visit_tree(tree, msg)
        return msg

    if c == 2:
        srv = ServiceSpecification(
            package_name, interface_name,
            MessageSpecification(
                package_name, namespace, interface_name + SERVICE_REQUEST_MESSAGE_SUFFIX, [], []),
            MessageSpecification(
                package_name, namespace, interface_name + SERVICE_RESPONSE_MESSAGE_SUFFIX, [], []))
        visit_tree(tree, srv)
        return srv

    assert False, 'Unsupported %d: %s' % (c, tree)


def count(tree, data):
    if tree.data == data:
        return 1
    c = 0
    for child in tree.children:
        if isinstance(child, Tree):
            c += count(child, data)
    return c


def visit_tree(tree, spec):
    if tree.data == 'struct_def':
        visit_struct_def(tree, spec)
    for c in tree.children:
        if isinstance(c, Tree):
            visit_tree(c, spec)


def type_from_const_type(child):
    '''Given a const_decl return the string idl type.'''
    assert 'const_type' == child.data
    const_type = child.children[0]
    assert len(const_type.children) == 1
    const_type = const_type.children[0]
    assert len(const_type.children) == 1
    const_type = const_type.children[0].data


def visit_struct_def(tree, spec):
    assert tree.data == 'struct_def'
    assert len(tree.children) >= 1
    c = tree.children[0]
    assert isinstance(c, Token)
    assert c.type == 'IDENTIFIER'
    if isinstance(spec, MessageSpecification):
        assert c.value == spec.msg_name
        msg = spec
    if isinstance(spec, ServiceSpecification):
        if c.value == spec.srv_name + SERVICE_REQUEST_MESSAGE_SUFFIX:
            msg = spec.request
        elif c.value == spec.srv_name + SERVICE_RESPONSE_MESSAGE_SUFFIX:
            msg = spec.response
        else:
            assert False

    for c in tree.children[1:]:
        if not isinstance(c, Tree):
            continue
        if c.data == 'member':
            visit_member(c, msg)


def get_scoped_name(tree):
    assert tree.data == 'scoped_name'
    scoped_name = []
    if len(tree.children) == 2:
        c = tree.children[0]
        assert c.data == 'scoped_name'
        scoped_name += get_scoped_name(c)
    c = tree.children[-1]
    assert isinstance(c, Token)
    assert c.type == 'IDENTIFIER'
    scoped_name.append(c.value)
    return scoped_name


def visit_member(tree, msg):
    assert tree.data == 'member'

    type_spec = None
    declarators = None
    annotation_applications = []
    for child in tree.children:
        if 'type_spec' == child.data:
            assert type_spec is None
            type_spec = child
        elif 'declarators' == child.data:
            assert declarators is None
            declarators = child
        else:
            assert 'annotation_appl' == child.data
            annotation_applications.append(child)

    if type_spec.data == 'simple_type_spec':
        assert len(type_spec.children) == 1
        type_spec = type_spec.children[0]
        assert type_spec.data == 'scoped_name'
        field_type = '::'.join(get_scoped_name(type_spec))
    else:
        field_type = None

    assert len(declarators.children) == 1
    c = declarators.children[0]
    assert c.data == 'declarator'
    assert len(c.children) == 1
    c = c.children[0]
    assert c.data == 'simple_declarator'
    assert len(c.children) == 1
    c = c.children[0]
    assert isinstance(c, Token)
    assert c.type == 'IDENTIFIER'
    field_name = c.value

    annotations = []
    for c in annotation_applications:
        sn = list(c.find_data('scoped_name'))[0]
        assert sn.data == 'scoped_name'
        assert len(sn.children) == 1
        sn = sn.children[0]
        assert isinstance(sn, Token)
        assert sn.type == 'IDENTIFIER'
        annotation_type = sn.value

        annotation_args = []
        for params in list(c.find_data('annotation_appl_params')):
            assert len(params.children) >= 1
            for param in params.children:
                assert param.data == 'annotation_appl_param'
                ident = param.children[0]
                assert isinstance(ident, Token)

                value = list(param.find_data('const_expr'))[0]
                value = reconstructor.reconstruct(value)
                annotation_args.append((ident.value, value))

        annotations.append((annotation_type, annotation_args))

    field_default_value = None
    for atype, args in annotations:
        # Silently ignore unsupported annotations
        if 'default' == atype:
            # Only allow one default annotation
            assert field_default_value is None
            assert len(args) == 1
            arg = args[0]
            assert 'value' == arg[0]
            field_default_value = arg[1]
        elif 'verbatim' == atype:
            if len(args) == 2:
                language = None
                text = None
                for arg in args:
                    if 'language' == arg[0]:
                        language = arg[1]
                    elif 'text' == arg[0]:
                        text = arg[1]

                if 'rosidl_array_init' == language:
                    assert field_default_value is None
                    field_default_value = text

    if field_type is not None:
        # TODO extract array typedefs from AST and resolve them correctly
        parts = field_type.split('__')
        try:
            if str(int(parts[-1])) == parts[-1]:
                field_type = '::'.join(parts[:-1]) + '[' + parts[-1] + ']'
        except ValueError:
            pass
        msg.fields.append(
            Field(
                Type(field_type, context_package_name=msg.base_type.pkg_name),
                field_name, default_value_string=field_default_value))
        # TODO(sloretz) msg.constants.append()


# Go through AST
# Keep a scope stack
# Collect all module and struct definitions
# Afterwards connect struct and struct_Constants module to make one message definition

def convert_ast(tree):
    """Convert parser AST into another intermediate representation."""
    # A stack that tracks naming
    scope_stack = [IDL()]

    for child in tree.children:
        assert isinstance(child, Tree) and 'definition' == child.data and 1 == len(child.children)
        descend_definition(child, scope_stack)
    return idl


def descend_definition(tree, scope_stack):
    assert isinstance(child, Tree) and 'definition' == child.data and len(child.children) == 1
    if 'const_dcl' == definition.children[0].data:
        descend_constant(definition.children[0], scope_stack)
    elif 'struct_def' == child.data:
        descend_struct(child, scope_stack)
    elif 'module_dcl' == child.data:
        descend_module(child, scope_stack)
    elif 'typedef_dcl' == child.data:
        raise NotImplementedError
    else:
        raise RuntimeError('Unexppected ast node ' + repr(child))


def descend_module(tree, scope_stack):
    assert 'module_dcl' == tree.data
    module = Module()
    idl = scope_stack[0]
    idl.modules.append(module)

    # skipping module annotations
    child_num = 0
    for child_num, child in enumerate(tree.children):
        if isinstance(child, Tree) and 'annotation_appl' == tree.data:
            continue
        break

    # Get the name of the module
    child = tree.children[child_num]
    child_num += 1
    assert isinstance(child, Token) and 'IDENTIFIER' == child.type
    module.idenitifer = scope_stack[-1].qualify_name(str(child))
    # Start a scope now that the name of it is known
    scope_stack.append(module)

    # Traverse definitions
    for child in tree.children[child_num:]:
        if isinstance(child, Tree) and 'definition' == child.data:
            definition = child.children[0]
            descend_definition(definition, scope_stack)
        else:
            raise RuntimeError('Unexppected ast node ' + repr(child))


def descend_struct(tree, scope_stack):
    assert tree.data == 'struct_def'
    struct = Struct()
    idl = scope_stack[0]
    idl.structs.append(struct)

    # skipping struct annotations
    child_num = 0
    for child_num, child in enumerate(tree.children):
        if isinstance(child, Tree) and 'annotation_appl' == tree.data:
            continue
        break

    # Get name of struct
    child = tree.children[child_num]
    child_num += 1
    assert isinstance(child, Token) and 'IDENTIFIER' == child.type
    struct.idenitifer = scope_stack[-1].qualify_name(str(child))
    # Start a scope now that the name of it is known
    scope_stack.append(struct)

    # Traverse struct members
    for child in tree.children[child_num:]:
        if isinstance(child, Tree) and 'member' == child.data:
            member = child.children[0]
            descend_member(member, scope_stack)
        else:
            raise RuntimeError('Unexppected ast node ' + repr(child))


def descend_member(tree, scope_stack):
    assert 'member' == tree.data
    member = Member()
    scope = scope_stack[-1]
    scope.members.append(member)

    # Go through annotations
    annotations = []
    child_num = 0
    for child_num, child in enumerate(tree.children):
        annotation = AnnotationAplication.from_annotation_appl(child)
        if 'default' == annotation.identifier:
            raise NotImplementedError
        elif 'verbatim' == annotation.identifier:
            raise NotImplementedError

    # Get member type
    type_spec = tree.children[child_num]
    if 'template_type_spec' == type_spec.children[0].data:
        raise NotImplementedError
    assert 'simple_type_spec' == type_spec.children[0].data
    simple_spec = type_spec.children[0]
    assert 'scoped_name' == simple_spec.children[0]
    member.type_specifier = scope.qualify_name('::'.join(get_scoped_name(type_spec)))

    # Get member name
    declartors = tree.children[child_num]
    if len(declarators.children) > 1:
        raise NotImplementedError('Only a single declarator per member is supported')
    declarator = declarators.children[0]
    assert 'declarator' == declarator.data
    declarator = declarator.children[0]
    if 'array_declarator' == declarator.data:
        raise NotImplementedError
    assert 'simple_declarator' == declarator.data
    assert isinstance(declarator.children[0], Token)
    member.identifier = declarator.children[0].value


def descend_constant(tree, scope_stack):
    scope = scope_stack[-1]
    constant = Constant()
    scope.constants.append(constant)
    assert 'const_dcl' == tree.data and len(tree.children) == 3
    assert 'const_type' == tree.children[0].data
    const_type = type_from_const_type(tree.children[0])
    assert 'IDENTIFIER' == tree.children[1].type
    const_name = scope.qualify_name(tree.children[1].children[0])
    assert 'const_expr' == tree.children[2].data
    const_expr = tree.children[2].data
    # TODO(sloretz) how to get meaninful data out of const_expr
    raise NotImplementedError


def join_name(base, scoped_name):
    return base + '::' + scoped_name


class Scope:

    def __init__(self, base_identifier = ""):
        # List of Typedef instances in this scope
        self.typedefs = []
        self.identifier = base_identifier

    def qualify_name(self, unqualified_name):
        # TODO(sloretz) does the scope stack need to be searched when the unqualified name has :: ?
        return join_name(self.identifier, unqualified_name)


class AnnotationApplication:

    @classmethod
    def from_annotation_appl(cls, tree):
        """Return parsed annotation application."""
        annotation = cls()
        assert 'annotation_appl' == tree.data
        assert 'scoped_name' == tree.children[0].data
        scoped_name = tree.children[0]
        annotation.identifier = scoped_name.children[0].value
        assert 'annotation_appl_params' == tree.children[1]
        params = tree.children[0]
        for param in params.children:
            if 'const_expr' == param.data:
                annotation.params.append((None, reconstructor.reconstruct(param)))
            elif 'annotation_appl_param' == param.data:
                identifier = param.children[0]
                assert isinstance(identifier, Token) and 'IDENTIFIER' == identifier.type
                value = param.children[1]
                assert isinstance(value, Tree) and 'const_expr' == value.data
                annotation.params.append(str(value), reconstructor.reconstruct(value))
        return annotation

    def __init__(self):
        self.identifier = None
        # 2-tuple (name, value)
        self.params = []


class Constant:

    def __init__(self):
        # Fully Qualified name
        self.identifier = None
        # Fully Qualified type
        self.type_identifier = None
        # Evaluated constant expression
        self.expression_result = None


class Member:

    def __init__(self):
        # Name of member
        self.identifier = None
        # Type of the member
        self.type_specifier = None
        # Default value
        self.default_value = None


class IDL(Scope):

    def __init__(self):
        # basename of the idl file
        self.name = None
        # List of all module instances at any level
        self.modules = []
        # List of all struct instances at any level
        self.structs = []
        # List of Constant intances only at the top level
        self.constants = []


class Module(Scope):

    def __init__(self):
        super().__init__()
        # TODO(sloretz) List of other names that identify this from typedefs
        # Fully qualified name
        self.identifier = None
        # List of Constant intances
        self.constants = []


class Struct(Scope):

    def __init__(self):
        super().__init__()
        # TODO(sloretz) List of other names that identify this from typedefs
        # Fully qualified name
        self.identifier = None
        # List of 3-tuple (type <str>, fully qualified name <str>, default value)
        self.members = []
