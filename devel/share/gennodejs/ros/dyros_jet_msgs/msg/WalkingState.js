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

class WalkingState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.walking_end = null;
      this.walking_end_foot_side = null;
    }
    else {
      if (initObj.hasOwnProperty('walking_end')) {
        this.walking_end = initObj.walking_end
      }
      else {
        this.walking_end = false;
      }
      if (initObj.hasOwnProperty('walking_end_foot_side')) {
        this.walking_end_foot_side = initObj.walking_end_foot_side
      }
      else {
        this.walking_end_foot_side = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WalkingState
    // Serialize message field [walking_end]
    bufferOffset = _serializer.bool(obj.walking_end, buffer, bufferOffset);
    // Serialize message field [walking_end_foot_side]
    bufferOffset = _serializer.bool(obj.walking_end_foot_side, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WalkingState
    let len;
    let data = new WalkingState(null);
    // Deserialize message field [walking_end]
    data.walking_end = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [walking_end_foot_side]
    data.walking_end_foot_side = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dyros_jet_msgs/WalkingState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e5b99df3c7eac3f1f42ddf9f1822ced4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool walking_end
    bool walking_end_foot_side
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WalkingState(null);
    if (msg.walking_end !== undefined) {
      resolved.walking_end = msg.walking_end;
    }
    else {
      resolved.walking_end = false
    }

    if (msg.walking_end_foot_side !== undefined) {
      resolved.walking_end_foot_side = msg.walking_end_foot_side;
    }
    else {
      resolved.walking_end_foot_side = false
    }

    return resolved;
    }
};

module.exports = WalkingState;
