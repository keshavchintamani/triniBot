// Auto-generated. Do not edit!

// (in-package balltracker.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------

class command {
  constructor() {
    this.priority = '';
    this.direction = '';
    this.speed = 0;
    this.angle = 0.0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type command
    // Serialize message field [priority]
    bufferInfo = _serializer.string(obj.priority, bufferInfo);
    // Serialize message field [direction]
    bufferInfo = _serializer.string(obj.direction, bufferInfo);
    // Serialize message field [speed]
    bufferInfo = _serializer.int32(obj.speed, bufferInfo);
    // Serialize message field [angle]
    bufferInfo = _serializer.float32(obj.angle, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type command
    let tmp;
    let len;
    let data = new command();
    // Deserialize message field [priority]
    tmp = _deserializer.string(buffer);
    data.priority = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [direction]
    tmp = _deserializer.string(buffer);
    data.direction = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [speed]
    tmp = _deserializer.int32(buffer);
    data.speed = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [angle]
    tmp = _deserializer.float32(buffer);
    data.angle = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'balltracker/command';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '42cf6ba923645af9d43237e750433163';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string priority
    string direction
    int32 speed
    float32 angle
    
    `;
  }

};

module.exports = command;
