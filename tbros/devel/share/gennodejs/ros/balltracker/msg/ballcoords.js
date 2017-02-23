// Auto-generated. Do not edit!

// (in-package balltracker.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------

class ballcoords {
  constructor() {
    this.color = '';
    this.x = 0.0;
    this.y = 0.0;
    this.radius = 0.0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type ballcoords
    // Serialize message field [color]
    bufferInfo = _serializer.string(obj.color, bufferInfo);
    // Serialize message field [x]
    bufferInfo = _serializer.float32(obj.x, bufferInfo);
    // Serialize message field [y]
    bufferInfo = _serializer.float32(obj.y, bufferInfo);
    // Serialize message field [radius]
    bufferInfo = _serializer.float32(obj.radius, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type ballcoords
    let tmp;
    let len;
    let data = new ballcoords();
    // Deserialize message field [color]
    tmp = _deserializer.string(buffer);
    data.color = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [x]
    tmp = _deserializer.float32(buffer);
    data.x = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [y]
    tmp = _deserializer.float32(buffer);
    data.y = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [radius]
    tmp = _deserializer.float32(buffer);
    data.radius = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'balltracker/ballcoords';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd5c701da1a7655c4c6cd03fc0658634a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string color
    float32 x
    float32 y
    float32 radius
    
    `;
  }

};

module.exports = ballcoords;
