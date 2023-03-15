# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: osi3/osi_roadmarking.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from osi3 import osi_common_pb2 as osi3_dot_osi__common__pb2
from osi3 import osi_trafficsign_pb2 as osi3_dot_osi__trafficsign__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='osi3/osi_roadmarking.proto',
  package='osi3',
  syntax='proto2',
  serialized_options=b'H\001',
  serialized_pb=b'\n\x1aosi3/osi_roadmarking.proto\x12\x04osi3\x1a\x15osi3/osi_common.proto\x1a\x1aosi3/osi_trafficsign.proto\"\xc8\x06\n\x0bRoadMarking\x12\x1c\n\x02id\x18\x01 \x01(\x0b\x32\x10.osi3.Identifier\x12\"\n\x04\x62\x61se\x18\x02 \x01(\x0b\x32\x14.osi3.BaseStationary\x12\x38\n\x0e\x63lassification\x18\x03 \x01(\x0b\x32 .osi3.RoadMarking.Classification\x1a\xbc\x05\n\x0e\x43lassification\x12\x33\n\x04type\x18\x01 \x01(\x0e\x32%.osi3.RoadMarking.Classification.Type\x12N\n\x16traffic_main_sign_type\x18\x02 \x01(\x0e\x32..osi3.TrafficSign.MainSign.Classification.Type\x12@\n\x10monochrome_color\x18\x03 \x01(\x0e\x32&.osi3.RoadMarking.Classification.Color\x12%\n\x05value\x18\x04 \x01(\x0b\x32\x16.osi3.TrafficSignValue\x12\x12\n\nvalue_text\x18\x05 \x01(\t\x12*\n\x10\x61ssigned_lane_id\x18\x06 \x03(\x0b\x32\x10.osi3.Identifier\x12\x19\n\x11is_out_of_service\x18\x07 \x01(\x08\"\xcd\x01\n\x04Type\x12\x10\n\x0cTYPE_UNKNOWN\x10\x00\x12\x0e\n\nTYPE_OTHER\x10\x01\x12\x1d\n\x19TYPE_PAINTED_TRAFFIC_SIGN\x10\x02\x12\x1e\n\x1aTYPE_SYMBOLIC_TRAFFIC_SIGN\x10\x03\x12\x1d\n\x19TYPE_TEXTUAL_TRAFFIC_SIGN\x10\x04\x12\x17\n\x13TYPE_GENERIC_SYMBOL\x10\x05\x12\x15\n\x11TYPE_GENERIC_LINE\x10\x06\x12\x15\n\x11TYPE_GENERIC_TEXT\x10\x07\"\x90\x01\n\x05\x43olor\x12\x11\n\rCOLOR_UNKNOWN\x10\x00\x12\x0f\n\x0b\x43OLOR_OTHER\x10\x01\x12\x0f\n\x0b\x43OLOR_WHITE\x10\x02\x12\x10\n\x0c\x43OLOR_YELLOW\x10\x03\x12\x0e\n\nCOLOR_BLUE\x10\x05\x12\r\n\tCOLOR_RED\x10\x06\x12\x0f\n\x0b\x43OLOR_GREEN\x10\x07\x12\x10\n\x0c\x43OLOR_VIOLET\x10\x08\x42\x02H\x01'
  ,
  dependencies=[osi3_dot_osi__common__pb2.DESCRIPTOR,osi3_dot_osi__trafficsign__pb2.DESCRIPTOR,])



_ROADMARKING_CLASSIFICATION_TYPE = _descriptor.EnumDescriptor(
  name='Type',
  full_name='osi3.RoadMarking.Classification.Type',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='TYPE_UNKNOWN', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TYPE_OTHER', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TYPE_PAINTED_TRAFFIC_SIGN', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TYPE_SYMBOLIC_TRAFFIC_SIGN', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TYPE_TEXTUAL_TRAFFIC_SIGN', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TYPE_GENERIC_SYMBOL', index=5, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TYPE_GENERIC_LINE', index=6, number=6,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='TYPE_GENERIC_TEXT', index=7, number=7,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=576,
  serialized_end=781,
)
_sym_db.RegisterEnumDescriptor(_ROADMARKING_CLASSIFICATION_TYPE)

_ROADMARKING_CLASSIFICATION_COLOR = _descriptor.EnumDescriptor(
  name='Color',
  full_name='osi3.RoadMarking.Classification.Color',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='COLOR_UNKNOWN', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='COLOR_OTHER', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='COLOR_WHITE', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='COLOR_YELLOW', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='COLOR_BLUE', index=4, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='COLOR_RED', index=5, number=6,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='COLOR_GREEN', index=6, number=7,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='COLOR_VIOLET', index=7, number=8,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=784,
  serialized_end=928,
)
_sym_db.RegisterEnumDescriptor(_ROADMARKING_CLASSIFICATION_COLOR)


_ROADMARKING_CLASSIFICATION = _descriptor.Descriptor(
  name='Classification',
  full_name='osi3.RoadMarking.Classification',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='type', full_name='osi3.RoadMarking.Classification.type', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='traffic_main_sign_type', full_name='osi3.RoadMarking.Classification.traffic_main_sign_type', index=1,
      number=2, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='monochrome_color', full_name='osi3.RoadMarking.Classification.monochrome_color', index=2,
      number=3, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='value', full_name='osi3.RoadMarking.Classification.value', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='value_text', full_name='osi3.RoadMarking.Classification.value_text', index=4,
      number=5, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='assigned_lane_id', full_name='osi3.RoadMarking.Classification.assigned_lane_id', index=5,
      number=6, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='is_out_of_service', full_name='osi3.RoadMarking.Classification.is_out_of_service', index=6,
      number=7, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _ROADMARKING_CLASSIFICATION_TYPE,
    _ROADMARKING_CLASSIFICATION_COLOR,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=228,
  serialized_end=928,
)

_ROADMARKING = _descriptor.Descriptor(
  name='RoadMarking',
  full_name='osi3.RoadMarking',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='osi3.RoadMarking.id', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='base', full_name='osi3.RoadMarking.base', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='classification', full_name='osi3.RoadMarking.classification', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[_ROADMARKING_CLASSIFICATION, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=88,
  serialized_end=928,
)

_ROADMARKING_CLASSIFICATION.fields_by_name['type'].enum_type = _ROADMARKING_CLASSIFICATION_TYPE
_ROADMARKING_CLASSIFICATION.fields_by_name['traffic_main_sign_type'].enum_type = osi3_dot_osi__trafficsign__pb2._TRAFFICSIGN_MAINSIGN_CLASSIFICATION_TYPE
_ROADMARKING_CLASSIFICATION.fields_by_name['monochrome_color'].enum_type = _ROADMARKING_CLASSIFICATION_COLOR
_ROADMARKING_CLASSIFICATION.fields_by_name['value'].message_type = osi3_dot_osi__trafficsign__pb2._TRAFFICSIGNVALUE
_ROADMARKING_CLASSIFICATION.fields_by_name['assigned_lane_id'].message_type = osi3_dot_osi__common__pb2._IDENTIFIER
_ROADMARKING_CLASSIFICATION.containing_type = _ROADMARKING
_ROADMARKING_CLASSIFICATION_TYPE.containing_type = _ROADMARKING_CLASSIFICATION
_ROADMARKING_CLASSIFICATION_COLOR.containing_type = _ROADMARKING_CLASSIFICATION
_ROADMARKING.fields_by_name['id'].message_type = osi3_dot_osi__common__pb2._IDENTIFIER
_ROADMARKING.fields_by_name['base'].message_type = osi3_dot_osi__common__pb2._BASESTATIONARY
_ROADMARKING.fields_by_name['classification'].message_type = _ROADMARKING_CLASSIFICATION
DESCRIPTOR.message_types_by_name['RoadMarking'] = _ROADMARKING
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

RoadMarking = _reflection.GeneratedProtocolMessageType('RoadMarking', (_message.Message,), {

  'Classification' : _reflection.GeneratedProtocolMessageType('Classification', (_message.Message,), {
    'DESCRIPTOR' : _ROADMARKING_CLASSIFICATION,
    '__module__' : 'osi3.osi_roadmarking_pb2'
    # @@protoc_insertion_point(class_scope:osi3.RoadMarking.Classification)
    })
  ,
  'DESCRIPTOR' : _ROADMARKING,
  '__module__' : 'osi3.osi_roadmarking_pb2'
  # @@protoc_insertion_point(class_scope:osi3.RoadMarking)
  })
_sym_db.RegisterMessage(RoadMarking)
_sym_db.RegisterMessage(RoadMarking.Classification)


DESCRIPTOR._options = None
# @@protoc_insertion_point(module_scope)
