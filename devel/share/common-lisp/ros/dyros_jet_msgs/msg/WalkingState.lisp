; Auto-generated. Do not edit!


(cl:in-package dyros_jet_msgs-msg)


;//! \htmlinclude WalkingState.msg.html

(cl:defclass <WalkingState> (roslisp-msg-protocol:ros-message)
  ((walking_end
    :reader walking_end
    :initarg :walking_end
    :type cl:boolean
    :initform cl:nil)
   (walking_end_foot_side
    :reader walking_end_foot_side
    :initarg :walking_end_foot_side
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass WalkingState (<WalkingState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WalkingState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WalkingState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dyros_jet_msgs-msg:<WalkingState> is deprecated: use dyros_jet_msgs-msg:WalkingState instead.")))

(cl:ensure-generic-function 'walking_end-val :lambda-list '(m))
(cl:defmethod walking_end-val ((m <WalkingState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dyros_jet_msgs-msg:walking_end-val is deprecated.  Use dyros_jet_msgs-msg:walking_end instead.")
  (walking_end m))

(cl:ensure-generic-function 'walking_end_foot_side-val :lambda-list '(m))
(cl:defmethod walking_end_foot_side-val ((m <WalkingState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dyros_jet_msgs-msg:walking_end_foot_side-val is deprecated.  Use dyros_jet_msgs-msg:walking_end_foot_side instead.")
  (walking_end_foot_side m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WalkingState>) ostream)
  "Serializes a message object of type '<WalkingState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'walking_end) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'walking_end_foot_side) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WalkingState>) istream)
  "Deserializes a message object of type '<WalkingState>"
    (cl:setf (cl:slot-value msg 'walking_end) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'walking_end_foot_side) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WalkingState>)))
  "Returns string type for a message object of type '<WalkingState>"
  "dyros_jet_msgs/WalkingState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WalkingState)))
  "Returns string type for a message object of type 'WalkingState"
  "dyros_jet_msgs/WalkingState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WalkingState>)))
  "Returns md5sum for a message object of type '<WalkingState>"
  "e5b99df3c7eac3f1f42ddf9f1822ced4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WalkingState)))
  "Returns md5sum for a message object of type 'WalkingState"
  "e5b99df3c7eac3f1f42ddf9f1822ced4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WalkingState>)))
  "Returns full string definition for message of type '<WalkingState>"
  (cl:format cl:nil "bool walking_end~%bool walking_end_foot_side~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WalkingState)))
  "Returns full string definition for message of type 'WalkingState"
  (cl:format cl:nil "bool walking_end~%bool walking_end_foot_side~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WalkingState>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WalkingState>))
  "Converts a ROS message object to a list"
  (cl:list 'WalkingState
    (cl:cons ':walking_end (walking_end msg))
    (cl:cons ':walking_end_foot_side (walking_end_foot_side msg))
))
