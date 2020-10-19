// Auto-generated. Do not edit!

// (in-package dyros_jet_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class JointState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.angle = null;
      this.velocity = null;
      this.current = null;
      this.error = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = [];
      }
      if (initObj.hasOwnProperty('angle')) {
        this.angle = initObj.angle
      }
      else {
        this.angle = [];
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = [];
      }
      if (initObj.hasOwnProperty('current')) {
        this.current = initObj.current
      }
      else {
        this.current = [];
      }
      if (initObj.hasOwnProperty('error')) {
        this.error = initObj.error
      }
      else {
        this.error = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointState
    // Serialize message field [name]
    bufferOffset = _arraySerializer.string(obj.name, buffer, bufferOffset, null);
    // Serialize message field [angle]
    bufferOffset = _arraySerializer.float64(obj.angle, buffer, bufferOffset, null);
    // Serialize message field [velocity]
    bufferOffset = _arraySerializer.float64(obj.velocity, buffer, bufferOffset, null);
    // Serialize message field [current]
    bufferOffset = _arraySerializer.float64(obj.current, buffer, bufferOffset, null);
    // Serialize message field [error]
    bufferOffset = _arraySerializer.uint32(obj.error, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointState
    let len;
    let data = new JointState(null);
    // Deserialize message field [name]
    data.name = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [angle]
    data.angle = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [velocity]
    data.velocity = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [current]
    data.current = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [error]
    data.error = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.name.forEach((val) => {
      length += 4 + val.length;
    });
    length += 8 * object.angle.length;
    length += 8 * object.velocity.length;
    length += 8 * object.current.length;
    length += 4 * object.error.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dyros_jet_msgs/JointState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '44f97ce7a113b09b2cda2af039ff08e1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    # DXL Error
    uint32 INPUT_VOLTAGE_ERROR	=1
    uint32 MOTOR_HALL_SENSOR_ERROR	=2
    uint32 OVERHEAT_ERROR		=4
    uint32 MOTOR_ENCODER_ERROR	=8
    uint32 ELECTRONICAL_SHOCK_ERROR	=16
    uint32 OVERLOAD_ERROR		=32
    uint32 RESERVED_ERROR		=64
    uint32 NOT_UPDATED		=128
    
    
    string[] name
    float64[] angle
    float64[] velocity
    float64[] current
    
    # bit 0000 0000 0000 0000 0000 0001 0000 0000 = Not updated
    # bit 0000 0000 0000 0000 0000 0000 0000 0100 = Not updated, Overheat error
    uint32[] error
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JointState(null);
    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = []
    }

    if (msg.angle !== undefined) {
      resolved.angle = msg.angle;
    }
    else {
      resolved.angle = []
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = []
    }

    if (msg.current !== undefined) {
      resolved.current = msg.current;
    }
    else {
      resolved.current = []
    }

    if (msg.error !== undefined) {
      resolved.error = msg.error;
    }
    else {
      resolved.error = []
    }

    return resolved;
    }
};

// Constants for message
JointState.Constants = {
  INPUT_VOLTAGE_ERROR: 1,
  MOTOR_HALL_SENSOR_ERROR: 2,
  OVERHEAT_ERROR: 4,
  MOTOR_ENCODER_ERROR: 8,
  ELECTRONICAL_SHOCK_ERROR: 16,
  OVERLOAD_ERROR: 32,
  RESERVED_ERROR: 64,
  NOT_UPDATED: 128,
}

module.exports = JointState;
