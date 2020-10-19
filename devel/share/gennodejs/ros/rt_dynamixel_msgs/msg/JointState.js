// Auto-generated. Do not edit!

// (in-package rt_dynamixel_msgs.msg)


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
      this.id = null;
      this.updated = null;
      this.angle = null;
      this.velocity = null;
      this.current = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = [];
      }
      if (initObj.hasOwnProperty('updated')) {
        this.updated = initObj.updated
      }
      else {
        this.updated = [];
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
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointState
    // Serialize message field [id]
    bufferOffset = _arraySerializer.uint8(obj.id, buffer, bufferOffset, null);
    // Serialize message field [updated]
    bufferOffset = _arraySerializer.uint8(obj.updated, buffer, bufferOffset, null);
    // Serialize message field [angle]
    bufferOffset = _arraySerializer.float64(obj.angle, buffer, bufferOffset, null);
    // Serialize message field [velocity]
    bufferOffset = _arraySerializer.float64(obj.velocity, buffer, bufferOffset, null);
    // Serialize message field [current]
    bufferOffset = _arraySerializer.float64(obj.current, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointState
    let len;
    let data = new JointState(null);
    // Deserialize message field [id]
    data.id = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [updated]
    data.updated = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [angle]
    data.angle = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [velocity]
    data.velocity = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [current]
    data.current = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.id.length;
    length += object.updated.length;
    length += 8 * object.angle.length;
    length += 8 * object.velocity.length;
    length += 8 * object.current.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rt_dynamixel_msgs/JointState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '06ba1356a0bc81b57a392e7b27a8749e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 UPDATED=0
    uint8 LOST=128
    
    uint8[] id
    uint8[] updated # error
    float64[] angle
    float64[] velocity
    float64[] current
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JointState(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = []
    }

    if (msg.updated !== undefined) {
      resolved.updated = msg.updated;
    }
    else {
      resolved.updated = []
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

    return resolved;
    }
};

// Constants for message
JointState.Constants = {
  UPDATED: 0,
  LOST: 128,
}

module.exports = JointState;
