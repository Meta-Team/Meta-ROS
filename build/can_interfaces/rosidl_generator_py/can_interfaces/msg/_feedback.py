# generated from rosidl_generator_py/resource/_idl.py.em
# with input from can_interfaces:msg/Feedback.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Feedback(type):
    """Metaclass of message 'Feedback'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('can_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'can_interfaces.msg.Feedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__feedback
            cls._TYPE_SUPPORT = module.type_support_msg__msg__feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__feedback

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Feedback(metaclass=Metaclass_Feedback):
    """Message class 'Feedback'."""

    __slots__ = [
        '_mst_id',
        '_id',
        '_pos',
        '_vel',
        '_tor',
        '_t_mos',
        '_t_rotor',
    ]

    _fields_and_field_types = {
        'mst_id': 'int8',
        'id': 'int8',
        'pos': 'float',
        'vel': 'float',
        'tor': 'float',
        't_mos': 'float',
        't_rotor': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.mst_id = kwargs.get('mst_id', int())
        self.id = kwargs.get('id', int())
        self.pos = kwargs.get('pos', float())
        self.vel = kwargs.get('vel', float())
        self.tor = kwargs.get('tor', float())
        self.t_mos = kwargs.get('t_mos', float())
        self.t_rotor = kwargs.get('t_rotor', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
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
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.mst_id != other.mst_id:
            return False
        if self.id != other.id:
            return False
        if self.pos != other.pos:
            return False
        if self.vel != other.vel:
            return False
        if self.tor != other.tor:
            return False
        if self.t_mos != other.t_mos:
            return False
        if self.t_rotor != other.t_rotor:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def mst_id(self):
        """Message field 'mst_id'."""
        return self._mst_id

    @mst_id.setter
    def mst_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mst_id' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'mst_id' field must be an integer in [-128, 127]"
        self._mst_id = value

    @property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'id' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'id' field must be an integer in [-128, 127]"
        self._id = value

    @property
    def pos(self):
        """Message field 'pos'."""
        return self._pos

    @pos.setter
    def pos(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pos' field must be of type 'float'"
        self._pos = value

    @property
    def vel(self):
        """Message field 'vel'."""
        return self._vel

    @vel.setter
    def vel(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vel' field must be of type 'float'"
        self._vel = value

    @property
    def tor(self):
        """Message field 'tor'."""
        return self._tor

    @tor.setter
    def tor(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tor' field must be of type 'float'"
        self._tor = value

    @property
    def t_mos(self):
        """Message field 't_mos'."""
        return self._t_mos

    @t_mos.setter
    def t_mos(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 't_mos' field must be of type 'float'"
        self._t_mos = value

    @property
    def t_rotor(self):
        """Message field 't_rotor'."""
        return self._t_rotor

    @t_rotor.setter
    def t_rotor(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 't_rotor' field must be of type 'float'"
        self._t_rotor = value
