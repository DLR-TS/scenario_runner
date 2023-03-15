# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: osi3/osi_datarecording.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from osi3 import osi_sensordata_pb2 as osi3_dot_osi__sensordata__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='osi3/osi_datarecording.proto',
  package='osi3',
  syntax='proto2',
  serialized_options=b'H\001',
  serialized_pb=b'\n\x1cosi3/osi_datarecording.proto\x12\x04osi3\x1a\x19osi3/osi_sensordata.proto\"9\n\x10SensorDataSeries\x12%\n\x0bsensor_data\x18\x01 \x03(\x0b\x32\x10.osi3.SensorData\">\n\x14SensorDataSeriesList\x12&\n\x06sensor\x18\x01 \x03(\x0b\x32\x16.osi3.SensorDataSeriesB\x02H\x01'
  ,
  dependencies=[osi3_dot_osi__sensordata__pb2.DESCRIPTOR,])




_SENSORDATASERIES = _descriptor.Descriptor(
  name='SensorDataSeries',
  full_name='osi3.SensorDataSeries',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='sensor_data', full_name='osi3.SensorDataSeries.sensor_data', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=65,
  serialized_end=122,
)


_SENSORDATASERIESLIST = _descriptor.Descriptor(
  name='SensorDataSeriesList',
  full_name='osi3.SensorDataSeriesList',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='sensor', full_name='osi3.SensorDataSeriesList.sensor', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=124,
  serialized_end=186,
)

_SENSORDATASERIES.fields_by_name['sensor_data'].message_type = osi3_dot_osi__sensordata__pb2._SENSORDATA
_SENSORDATASERIESLIST.fields_by_name['sensor'].message_type = _SENSORDATASERIES
DESCRIPTOR.message_types_by_name['SensorDataSeries'] = _SENSORDATASERIES
DESCRIPTOR.message_types_by_name['SensorDataSeriesList'] = _SENSORDATASERIESLIST
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

SensorDataSeries = _reflection.GeneratedProtocolMessageType('SensorDataSeries', (_message.Message,), {
  'DESCRIPTOR' : _SENSORDATASERIES,
  '__module__' : 'osi3.osi_datarecording_pb2'
  # @@protoc_insertion_point(class_scope:osi3.SensorDataSeries)
  })
_sym_db.RegisterMessage(SensorDataSeries)

SensorDataSeriesList = _reflection.GeneratedProtocolMessageType('SensorDataSeriesList', (_message.Message,), {
  'DESCRIPTOR' : _SENSORDATASERIESLIST,
  '__module__' : 'osi3.osi_datarecording_pb2'
  # @@protoc_insertion_point(class_scope:osi3.SensorDataSeriesList)
  })
_sym_db.RegisterMessage(SensorDataSeriesList)


DESCRIPTOR._options = None
# @@protoc_insertion_point(module_scope)
