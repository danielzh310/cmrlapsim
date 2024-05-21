# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: proto/vehicle_config.proto
"""Generated protocol buffer code."""
from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x1aproto/vehicle_config.proto\"V\n\nTireConfig\x12\x1b\n\x08tireType\x18\x01 \x01(\x0e\x32\t.TireType\x12\x15\n\rlongKnockdown\x18\x02 \x01(\x01\x12\x14\n\x0clatKnockdown\x18\x03 \x01(\x01\"k\n\x11\x41\x63\x63umulatorConfig\x12\x1b\n\x08\x63\x65llType\x18\x01 \x01(\x0e\x32\t.CellType\x12\x0f\n\x07nSeries\x18\x02 \x01(\x01\x12\x11\n\tnParallel\x18\x03 \x01(\x01\x12\x15\n\rstateOfCharge\x18\x04 \x01(\x01\"\xbd\x01\n\x10PowertrainConfig\x12-\n\x11\x61\x63\x63umulatorConfig\x18\x01 \x01(\x0b\x32\x12.AccumulatorConfig\x12\x1f\n\ntireConfig\x18\n \x01(\x0b\x32\x0b.TireConfig\x12\x1d\n\tmotorType\x18\x14 \x01(\x0e\x32\n.MotorType\x12\x12\n\nfrontRatio\x18\x15 \x01(\x01\x12\x11\n\trearRatio\x18\x16 \x01(\x01\x12\x13\n\x0binverterEff\x18\x17 \x01(\x01\"V\n\nAeroConfig\x12\x0c\n\x04\x41\x43_d\x18\x01 \x01(\x01\x12\x0c\n\x04\x41\x43_l\x18\x02 \x01(\x01\x12\x12\n\ndragHeight\x18\x03 \x01(\x01\x12\x18\n\x10rearAeroFraction\x18\x04 \x01(\x01\"Y\n\rAeroConfigDrs\x12\x0c\n\x04\x41\x43_d\x18\x01 \x01(\x01\x12\x0c\n\x04\x41\x43_l\x18\x02 \x01(\x01\x12\x12\n\ndragHeight\x18\x03 \x01(\x01\x12\x18\n\x10rearAeroFraction\x18\x04 \x01(\x01\"\x90\x01\n\x0eMassProperties\x12\x13\n\x0bvehicleMass\x18\x01 \x01(\x01\x12\x12\n\ndriverMass\x18\x02 \x01(\x01\x12\x15\n\rcornerInertia\x18\x03 \x01(\x01\x12\x12\n\nyawInertia\x18\x04 \x01(\x01\x12\x18\n\x10rearMassFraction\x18\x05 \x01(\x01\x12\x10\n\x08\x63gHeight\x18\x06 \x01(\x01\"\\\n\x10SuspensionConfig\x12\x11\n\twheelBase\x18\n \x01(\x01\x12\x12\n\ntrackWidth\x18\x0b \x01(\x01\x12!\n\x19rearRollStiffnessFraction\x18\x0c \x01(\x01\"\xdb\x01\n\rVehicleConfig\x12\'\n\x0emassProperties\x18\x01 \x01(\x0b\x32\x0f.MassProperties\x12+\n\x10suspensionConfig\x18\x02 \x01(\x0b\x32\x11.SuspensionConfig\x12+\n\x10powertrainConfig\x18\x03 \x01(\x0b\x32\x11.PowertrainConfig\x12\x1f\n\naeroConfig\x18\x04 \x01(\x0b\x32\x0b.AeroConfig\x12&\n\x0e\x61\x65roConfig_drs\x18\x05 \x01(\x0b\x32\x0e.AeroConfigDrs*\x1b\n\x08TireType\x12\x0f\n\x0bR25B_18x7_5\x10\x00*\x14\n\tMotorType\x12\x07\n\x03\x41MK\x10\x00*%\n\x08\x43\x65llType\x12\x08\n\x04VTC6\x10\x00\x12\x0f\n\x0bPouchCell52\x10\x01\x62\x06proto3')

_TIRETYPE = DESCRIPTOR.enum_types_by_name['TireType']
TireType = enum_type_wrapper.EnumTypeWrapper(_TIRETYPE)
_MOTORTYPE = DESCRIPTOR.enum_types_by_name['MotorType']
MotorType = enum_type_wrapper.EnumTypeWrapper(_MOTORTYPE)
_CELLTYPE = DESCRIPTOR.enum_types_by_name['CellType']
CellType = enum_type_wrapper.EnumTypeWrapper(_CELLTYPE)
R25B_18x7_5 = 0
AMK = 0
VTC6 = 0
PouchCell52 = 1


_TIRECONFIG = DESCRIPTOR.message_types_by_name['TireConfig']
_ACCUMULATORCONFIG = DESCRIPTOR.message_types_by_name['AccumulatorConfig']
_POWERTRAINCONFIG = DESCRIPTOR.message_types_by_name['PowertrainConfig']
_AEROCONFIG = DESCRIPTOR.message_types_by_name['AeroConfig']
_AEROCONFIGDRS = DESCRIPTOR.message_types_by_name['AeroConfigDrs']
_MASSPROPERTIES = DESCRIPTOR.message_types_by_name['MassProperties']
_SUSPENSIONCONFIG = DESCRIPTOR.message_types_by_name['SuspensionConfig']
_VEHICLECONFIG = DESCRIPTOR.message_types_by_name['VehicleConfig']
TireConfig = _reflection.GeneratedProtocolMessageType('TireConfig', (_message.Message,), {
  'DESCRIPTOR' : _TIRECONFIG,
  '__module__' : 'proto.vehicle_config_pb2'
  # @@protoc_insertion_point(class_scope:TireConfig)
  })
_sym_db.RegisterMessage(TireConfig)

AccumulatorConfig = _reflection.GeneratedProtocolMessageType('AccumulatorConfig', (_message.Message,), {
  'DESCRIPTOR' : _ACCUMULATORCONFIG,
  '__module__' : 'proto.vehicle_config_pb2'
  # @@protoc_insertion_point(class_scope:AccumulatorConfig)
  })
_sym_db.RegisterMessage(AccumulatorConfig)

PowertrainConfig = _reflection.GeneratedProtocolMessageType('PowertrainConfig', (_message.Message,), {
  'DESCRIPTOR' : _POWERTRAINCONFIG,
  '__module__' : 'proto.vehicle_config_pb2'
  # @@protoc_insertion_point(class_scope:PowertrainConfig)
  })
_sym_db.RegisterMessage(PowertrainConfig)

AeroConfig = _reflection.GeneratedProtocolMessageType('AeroConfig', (_message.Message,), {
  'DESCRIPTOR' : _AEROCONFIG,
  '__module__' : 'proto.vehicle_config_pb2'
  # @@protoc_insertion_point(class_scope:AeroConfig)
  })
_sym_db.RegisterMessage(AeroConfig)

AeroConfigDrs = _reflection.GeneratedProtocolMessageType('AeroConfigDrs', (_message.Message,), {
  'DESCRIPTOR' : _AEROCONFIGDRS,
  '__module__' : 'proto.vehicle_config_pb2'
  # @@protoc_insertion_point(class_scope:AeroConfigDrs)
  })
_sym_db.RegisterMessage(AeroConfigDrs)

MassProperties = _reflection.GeneratedProtocolMessageType('MassProperties', (_message.Message,), {
  'DESCRIPTOR' : _MASSPROPERTIES,
  '__module__' : 'proto.vehicle_config_pb2'
  # @@protoc_insertion_point(class_scope:MassProperties)
  })
_sym_db.RegisterMessage(MassProperties)

SuspensionConfig = _reflection.GeneratedProtocolMessageType('SuspensionConfig', (_message.Message,), {
  'DESCRIPTOR' : _SUSPENSIONCONFIG,
  '__module__' : 'proto.vehicle_config_pb2'
  # @@protoc_insertion_point(class_scope:SuspensionConfig)
  })
_sym_db.RegisterMessage(SuspensionConfig)

VehicleConfig = _reflection.GeneratedProtocolMessageType('VehicleConfig', (_message.Message,), {
  'DESCRIPTOR' : _VEHICLECONFIG,
  '__module__' : 'proto.vehicle_config_pb2'
  # @@protoc_insertion_point(class_scope:VehicleConfig)
  })
_sym_db.RegisterMessage(VehicleConfig)

if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _TIRETYPE._serialized_start=1061
  _TIRETYPE._serialized_end=1088
  _MOTORTYPE._serialized_start=1090
  _MOTORTYPE._serialized_end=1110
  _CELLTYPE._serialized_start=1112
  _CELLTYPE._serialized_end=1149
  _TIRECONFIG._serialized_start=30
  _TIRECONFIG._serialized_end=116
  _ACCUMULATORCONFIG._serialized_start=118
  _ACCUMULATORCONFIG._serialized_end=225
  _POWERTRAINCONFIG._serialized_start=228
  _POWERTRAINCONFIG._serialized_end=417
  _AEROCONFIG._serialized_start=419
  _AEROCONFIG._serialized_end=505
  _AEROCONFIGDRS._serialized_start=507
  _AEROCONFIGDRS._serialized_end=596
  _MASSPROPERTIES._serialized_start=599
  _MASSPROPERTIES._serialized_end=743
  _SUSPENSIONCONFIG._serialized_start=745
  _SUSPENSIONCONFIG._serialized_end=837
  _VEHICLECONFIG._serialized_start=840
  _VEHICLECONFIG._serialized_end=1059
# @@protoc_insertion_point(module_scope)
