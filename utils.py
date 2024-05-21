from google.protobuf import text_format

__all__ = [
    "writeProtoText",
   "readProtoText",
   "getEffectiveLinearMass",
   ]

def writeProtoText(filename, message):
    with open(filename, "w") as f:
        f.write(text_format.MessageToString(message))

def readProtoText(filename, messageType):
    message = messageType()
    with open(filename, "r") as f:
        text_format.Parse(f.read(), message)
    return message

def getEffectiveLinearMass(car):
    return car.m + 4 * car.massProperties.cornerInertia / car.powertrain.tire.radius**2