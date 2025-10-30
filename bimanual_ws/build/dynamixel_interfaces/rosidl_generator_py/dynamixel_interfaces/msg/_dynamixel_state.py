# generated from rosidl_generator_py/resource/_idl.py.em
# with input from dynamixel_interfaces:msg/DynamixelState.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

# Member 'id'
# Member 'dxl_hw_state'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DynamixelState(type):
    """Metaclass of message 'DynamixelState'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'COMM_STATE_OK': 0,
        'COMM_STATE_CANNOT_FIND_CONTROL_ITEM': -1,
        'COMM_STATE_OPEN_PORT_FAIL': -2,
        'COMM_STATE_INDIRECT_ADDR_FAIL': -3,
        'COMM_STATE_ITEM_WRITE_FAIL': -4,
        'COMM_STATE_ITEM_READ_FAIL': -5,
        'COMM_STATE_SYNC_WRITE_FAIL': -6,
        'COMM_STATE_SYNC_READ_FAIL': -7,
        'COMM_STATE_SET_SYNC_WRITE_FAIL': -8,
        'COMM_STATE_SET_SYNC_READ_FAIL': -9,
        'COMM_STATE_BULK_WRITE_FAIL': -10,
        'COMM_STATE_BULK_READ_FAIL': -11,
        'COMM_STATE_SET_BULK_WRITE_FAIL': -12,
        'COMM_STATE_SET_BULK_READ_FAIL': -13,
        'COMM_STATE_SET_READ_ITEM_FAIL': -14,
        'COMM_STATE_SET_WRITE_ITEM_FAIL': -15,
        'COMM_STATE_DXL_HARDWARE_ERROR': -16,
        'COMM_STATE_DXL_REBOOT_FAIL': -17,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('dynamixel_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'dynamixel_interfaces.msg.DynamixelState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__dynamixel_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__dynamixel_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__dynamixel_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__dynamixel_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__dynamixel_state

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'COMM_STATE_OK': cls.__constants['COMM_STATE_OK'],
            'COMM_STATE_CANNOT_FIND_CONTROL_ITEM': cls.__constants['COMM_STATE_CANNOT_FIND_CONTROL_ITEM'],
            'COMM_STATE_OPEN_PORT_FAIL': cls.__constants['COMM_STATE_OPEN_PORT_FAIL'],
            'COMM_STATE_INDIRECT_ADDR_FAIL': cls.__constants['COMM_STATE_INDIRECT_ADDR_FAIL'],
            'COMM_STATE_ITEM_WRITE_FAIL': cls.__constants['COMM_STATE_ITEM_WRITE_FAIL'],
            'COMM_STATE_ITEM_READ_FAIL': cls.__constants['COMM_STATE_ITEM_READ_FAIL'],
            'COMM_STATE_SYNC_WRITE_FAIL': cls.__constants['COMM_STATE_SYNC_WRITE_FAIL'],
            'COMM_STATE_SYNC_READ_FAIL': cls.__constants['COMM_STATE_SYNC_READ_FAIL'],
            'COMM_STATE_SET_SYNC_WRITE_FAIL': cls.__constants['COMM_STATE_SET_SYNC_WRITE_FAIL'],
            'COMM_STATE_SET_SYNC_READ_FAIL': cls.__constants['COMM_STATE_SET_SYNC_READ_FAIL'],
            'COMM_STATE_BULK_WRITE_FAIL': cls.__constants['COMM_STATE_BULK_WRITE_FAIL'],
            'COMM_STATE_BULK_READ_FAIL': cls.__constants['COMM_STATE_BULK_READ_FAIL'],
            'COMM_STATE_SET_BULK_WRITE_FAIL': cls.__constants['COMM_STATE_SET_BULK_WRITE_FAIL'],
            'COMM_STATE_SET_BULK_READ_FAIL': cls.__constants['COMM_STATE_SET_BULK_READ_FAIL'],
            'COMM_STATE_SET_READ_ITEM_FAIL': cls.__constants['COMM_STATE_SET_READ_ITEM_FAIL'],
            'COMM_STATE_SET_WRITE_ITEM_FAIL': cls.__constants['COMM_STATE_SET_WRITE_ITEM_FAIL'],
            'COMM_STATE_DXL_HARDWARE_ERROR': cls.__constants['COMM_STATE_DXL_HARDWARE_ERROR'],
            'COMM_STATE_DXL_REBOOT_FAIL': cls.__constants['COMM_STATE_DXL_REBOOT_FAIL'],
        }

    @property
    def COMM_STATE_OK(self):
        """Message constant 'COMM_STATE_OK'."""
        return Metaclass_DynamixelState.__constants['COMM_STATE_OK']

    @property
    def COMM_STATE_CANNOT_FIND_CONTROL_ITEM(self):
        """Message constant 'COMM_STATE_CANNOT_FIND_CONTROL_ITEM'."""
        return Metaclass_DynamixelState.__constants['COMM_STATE_CANNOT_FIND_CONTROL_ITEM']

    @property
    def COMM_STATE_OPEN_PORT_FAIL(self):
        """Message constant 'COMM_STATE_OPEN_PORT_FAIL'."""
        return Metaclass_DynamixelState.__constants['COMM_STATE_OPEN_PORT_FAIL']

    @property
    def COMM_STATE_INDIRECT_ADDR_FAIL(self):
        """Message constant 'COMM_STATE_INDIRECT_ADDR_FAIL'."""
        return Metaclass_DynamixelState.__constants['COMM_STATE_INDIRECT_ADDR_FAIL']

    @property
    def COMM_STATE_ITEM_WRITE_FAIL(self):
        """Message constant 'COMM_STATE_ITEM_WRITE_FAIL'."""
        return Metaclass_DynamixelState.__constants['COMM_STATE_ITEM_WRITE_FAIL']

    @property
    def COMM_STATE_ITEM_READ_FAIL(self):
        """Message constant 'COMM_STATE_ITEM_READ_FAIL'."""
        return Metaclass_DynamixelState.__constants['COMM_STATE_ITEM_READ_FAIL']

    @property
    def COMM_STATE_SYNC_WRITE_FAIL(self):
        """Message constant 'COMM_STATE_SYNC_WRITE_FAIL'."""
        return Metaclass_DynamixelState.__constants['COMM_STATE_SYNC_WRITE_FAIL']

    @property
    def COMM_STATE_SYNC_READ_FAIL(self):
        """Message constant 'COMM_STATE_SYNC_READ_FAIL'."""
        return Metaclass_DynamixelState.__constants['COMM_STATE_SYNC_READ_FAIL']

    @property
    def COMM_STATE_SET_SYNC_WRITE_FAIL(self):
        """Message constant 'COMM_STATE_SET_SYNC_WRITE_FAIL'."""
        return Metaclass_DynamixelState.__constants['COMM_STATE_SET_SYNC_WRITE_FAIL']

    @property
    def COMM_STATE_SET_SYNC_READ_FAIL(self):
        """Message constant 'COMM_STATE_SET_SYNC_READ_FAIL'."""
        return Metaclass_DynamixelState.__constants['COMM_STATE_SET_SYNC_READ_FAIL']

    @property
    def COMM_STATE_BULK_WRITE_FAIL(self):
        """Message constant 'COMM_STATE_BULK_WRITE_FAIL'."""
        return Metaclass_DynamixelState.__constants['COMM_STATE_BULK_WRITE_FAIL']

    @property
    def COMM_STATE_BULK_READ_FAIL(self):
        """Message constant 'COMM_STATE_BULK_READ_FAIL'."""
        return Metaclass_DynamixelState.__constants['COMM_STATE_BULK_READ_FAIL']

    @property
    def COMM_STATE_SET_BULK_WRITE_FAIL(self):
        """Message constant 'COMM_STATE_SET_BULK_WRITE_FAIL'."""
        return Metaclass_DynamixelState.__constants['COMM_STATE_SET_BULK_WRITE_FAIL']

    @property
    def COMM_STATE_SET_BULK_READ_FAIL(self):
        """Message constant 'COMM_STATE_SET_BULK_READ_FAIL'."""
        return Metaclass_DynamixelState.__constants['COMM_STATE_SET_BULK_READ_FAIL']

    @property
    def COMM_STATE_SET_READ_ITEM_FAIL(self):
        """Message constant 'COMM_STATE_SET_READ_ITEM_FAIL'."""
        return Metaclass_DynamixelState.__constants['COMM_STATE_SET_READ_ITEM_FAIL']

    @property
    def COMM_STATE_SET_WRITE_ITEM_FAIL(self):
        """Message constant 'COMM_STATE_SET_WRITE_ITEM_FAIL'."""
        return Metaclass_DynamixelState.__constants['COMM_STATE_SET_WRITE_ITEM_FAIL']

    @property
    def COMM_STATE_DXL_HARDWARE_ERROR(self):
        """Message constant 'COMM_STATE_DXL_HARDWARE_ERROR'."""
        return Metaclass_DynamixelState.__constants['COMM_STATE_DXL_HARDWARE_ERROR']

    @property
    def COMM_STATE_DXL_REBOOT_FAIL(self):
        """Message constant 'COMM_STATE_DXL_REBOOT_FAIL'."""
        return Metaclass_DynamixelState.__constants['COMM_STATE_DXL_REBOOT_FAIL']


class DynamixelState(metaclass=Metaclass_DynamixelState):
    """
    Message class 'DynamixelState'.

    Constants:
      COMM_STATE_OK
      COMM_STATE_CANNOT_FIND_CONTROL_ITEM
      COMM_STATE_OPEN_PORT_FAIL
      COMM_STATE_INDIRECT_ADDR_FAIL
      COMM_STATE_ITEM_WRITE_FAIL
      COMM_STATE_ITEM_READ_FAIL
      COMM_STATE_SYNC_WRITE_FAIL
      COMM_STATE_SYNC_READ_FAIL
      COMM_STATE_SET_SYNC_WRITE_FAIL
      COMM_STATE_SET_SYNC_READ_FAIL
      COMM_STATE_BULK_WRITE_FAIL
      COMM_STATE_BULK_READ_FAIL
      COMM_STATE_SET_BULK_WRITE_FAIL
      COMM_STATE_SET_BULK_READ_FAIL
      COMM_STATE_SET_READ_ITEM_FAIL
      COMM_STATE_SET_WRITE_ITEM_FAIL
      COMM_STATE_DXL_HARDWARE_ERROR
      COMM_STATE_DXL_REBOOT_FAIL
    """

    __slots__ = [
        '_header',
        '_comm_state',
        '_id',
        '_torque_state',
        '_dxl_hw_state',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'comm_state': 'int32',
        'id': 'sequence<int32>',
        'torque_state': 'sequence<boolean>',
        'dxl_hw_state': 'sequence<int32>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('boolean')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.comm_state = kwargs.get('comm_state', int())
        self.id = array.array('i', kwargs.get('id', []))
        self.torque_state = kwargs.get('torque_state', [])
        self.dxl_hw_state = array.array('i', kwargs.get('dxl_hw_state', []))

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.comm_state != other.comm_state:
            return False
        if self.id != other.id:
            return False
        if self.torque_state != other.torque_state:
            return False
        if self.dxl_hw_state != other.dxl_hw_state:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if self._check_fields:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def comm_state(self):
        """Message field 'comm_state'."""
        return self._comm_state

    @comm_state.setter
    def comm_state(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'comm_state' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'comm_state' field must be an integer in [-2147483648, 2147483647]"
        self._comm_state = value

    @builtins.property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'i', \
                    "The 'id' array.array() must have the type code of 'i'"
                self._id = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'id' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._id = array.array('i', value)

    @builtins.property
    def torque_state(self):
        """Message field 'torque_state'."""
        return self._torque_state

    @torque_state.setter
    def torque_state(self, value):
        if self._check_fields:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, bool) for v in value) and
                 True), \
                "The 'torque_state' field must be a set or sequence and each value of type 'bool'"
        self._torque_state = value

    @builtins.property
    def dxl_hw_state(self):
        """Message field 'dxl_hw_state'."""
        return self._dxl_hw_state

    @dxl_hw_state.setter
    def dxl_hw_state(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'i', \
                    "The 'dxl_hw_state' array.array() must have the type code of 'i'"
                self._dxl_hw_state = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'dxl_hw_state' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._dxl_hw_state = array.array('i', value)
