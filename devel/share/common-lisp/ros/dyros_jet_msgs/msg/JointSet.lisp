; Auto-generated. Do not edit!


(cl:in-package dyros_jet_msgs-msg)


;//! \htmlinclude JointSet.msg.html

(cl:defclass <JointSet> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass JointSet (<JointSet>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointSet>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointSet)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dyros_jet_msgs-msg:<JointSet> is deprecated: use dyros_jet_msgs-msg:JointSet instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <JointSet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dyros_jet_msgs-msg:id-val is deprecated.  Use dyros_jet_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <JointSet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dyros_jet_msgs-msg:angle-val is deprecated.  Use dyros_jet_msgs-msg:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointSet>) ostream)
  "Serializes a message object of type '<JointSet>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointSet>) istream)
  "Deserializes a message object of type '<JointSet>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointSet>)))
  "Returns string type for a message object of type '<JointSet>"
  "dyros_jet_msgs/JointSet")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointSet)))
  "Returns string type for a message object of type 'JointSet"
  "dyros_jet_msgs/JointSet")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointSet>)))
  "Returns md5sum for a message object of type '<JointSet>"
  "9c94549c9e988fc6422291c5c0b1ea5b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointSet)))
  "Returns md5sum for a message object of type 'JointSet"
  "9c94549c9e988fc6422291c5c0b1ea5b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointSet>)))
  "Returns full string definition for message of type '<JointSet>"
  (cl:format cl:nil "uint8 id~%float64 angle~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointSet)))
  "Returns full string definition for message of type 'JointSet"
  (cl:format cl:nil "uint8 id~%float64 angle~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointSet>))
  (cl:+ 0
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointSet>))
  "Converts a ROS message object to a list"
  (cl:list 'JointSet
    (cl:cons ':id (id msg))
    (cl:cons ':angle (angle msg))
))
