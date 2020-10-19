; Auto-generated. Do not edit!


(cl:in-package dyros_jet_msgs-msg)


;//! \htmlinclude WalkingCommand.msg.html

(cl:defclass <WalkingCommand> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (walk_mode
    :reader walk_mode
    :initarg :walk_mode
    :type cl:integer
    :initform 0)
   (compensator_mode
    :reader compensator_mode
    :initarg :compensator_mode
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 2 :element-type 'cl:boolean :initial-element cl:nil))
   (ik_mode
    :reader ik_mode
    :initarg :ik_mode
    :type cl:integer
    :initform 0)
   (heel_toe
    :reader heel_toe
    :initarg :heel_toe
    :type cl:boolean
    :initform cl:nil)
   (first_foot_step
    :reader first_foot_step
    :initarg :first_foot_step
    :type cl:boolean
    :initform cl:nil)
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (height
    :reader height
    :initarg :height
    :type cl:float
    :initform 0.0)
   (theta
    :reader theta
    :initarg :theta
    :type cl:float
    :initform 0.0)
   (step_length_x
    :reader step_length_x
    :initarg :step_length_x
    :type cl:float
    :initform 0.0)
   (step_length_y
    :reader step_length_y
    :initarg :step_length_y
    :type cl:float
    :initform 0.0)
   (walking_pattern
    :reader walking_pattern
    :initarg :walking_pattern
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass WalkingCommand (<WalkingCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WalkingCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WalkingCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dyros_jet_msgs-msg:<WalkingCommand> is deprecated: use dyros_jet_msgs-msg:WalkingCommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <WalkingCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dyros_jet_msgs-msg:header-val is deprecated.  Use dyros_jet_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'walk_mode-val :lambda-list '(m))
(cl:defmethod walk_mode-val ((m <WalkingCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dyros_jet_msgs-msg:walk_mode-val is deprecated.  Use dyros_jet_msgs-msg:walk_mode instead.")
  (walk_mode m))

(cl:ensure-generic-function 'compensator_mode-val :lambda-list '(m))
(cl:defmethod compensator_mode-val ((m <WalkingCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dyros_jet_msgs-msg:compensator_mode-val is deprecated.  Use dyros_jet_msgs-msg:compensator_mode instead.")
  (compensator_mode m))

(cl:ensure-generic-function 'ik_mode-val :lambda-list '(m))
(cl:defmethod ik_mode-val ((m <WalkingCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dyros_jet_msgs-msg:ik_mode-val is deprecated.  Use dyros_jet_msgs-msg:ik_mode instead.")
  (ik_mode m))

(cl:ensure-generic-function 'heel_toe-val :lambda-list '(m))
(cl:defmethod heel_toe-val ((m <WalkingCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dyros_jet_msgs-msg:heel_toe-val is deprecated.  Use dyros_jet_msgs-msg:heel_toe instead.")
  (heel_toe m))

(cl:ensure-generic-function 'first_foot_step-val :lambda-list '(m))
(cl:defmethod first_foot_step-val ((m <WalkingCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dyros_jet_msgs-msg:first_foot_step-val is deprecated.  Use dyros_jet_msgs-msg:first_foot_step instead.")
  (first_foot_step m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <WalkingCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dyros_jet_msgs-msg:x-val is deprecated.  Use dyros_jet_msgs-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <WalkingCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dyros_jet_msgs-msg:y-val is deprecated.  Use dyros_jet_msgs-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <WalkingCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dyros_jet_msgs-msg:z-val is deprecated.  Use dyros_jet_msgs-msg:z instead.")
  (z m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <WalkingCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dyros_jet_msgs-msg:height-val is deprecated.  Use dyros_jet_msgs-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <WalkingCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dyros_jet_msgs-msg:theta-val is deprecated.  Use dyros_jet_msgs-msg:theta instead.")
  (theta m))

(cl:ensure-generic-function 'step_length_x-val :lambda-list '(m))
(cl:defmethod step_length_x-val ((m <WalkingCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dyros_jet_msgs-msg:step_length_x-val is deprecated.  Use dyros_jet_msgs-msg:step_length_x instead.")
  (step_length_x m))

(cl:ensure-generic-function 'step_length_y-val :lambda-list '(m))
(cl:defmethod step_length_y-val ((m <WalkingCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dyros_jet_msgs-msg:step_length_y-val is deprecated.  Use dyros_jet_msgs-msg:step_length_y instead.")
  (step_length_y m))

(cl:ensure-generic-function 'walking_pattern-val :lambda-list '(m))
(cl:defmethod walking_pattern-val ((m <WalkingCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dyros_jet_msgs-msg:walking_pattern-val is deprecated.  Use dyros_jet_msgs-msg:walking_pattern instead.")
  (walking_pattern m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<WalkingCommand>)))
    "Constants for message type '<WalkingCommand>"
  '((:NONE . 0)
    (:STATIC_WALKING . 1)
    (:HIP_COMPENSTOR . 0)
    (:EXTERNAL_ENCODER . 1)
    (:IK . 0)
    (:JACOBIAN . 1)
    (:QP . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'WalkingCommand)))
    "Constants for message type 'WalkingCommand"
  '((:NONE . 0)
    (:STATIC_WALKING . 1)
    (:HIP_COMPENSTOR . 0)
    (:EXTERNAL_ENCODER . 1)
    (:IK . 0)
    (:JACOBIAN . 1)
    (:QP . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WalkingCommand>) ostream)
  "Serializes a message object of type '<WalkingCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'walk_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'walk_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'walk_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'walk_mode)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'compensator_mode))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ik_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ik_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ik_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ik_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'heel_toe) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'first_foot_step) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'height))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'step_length_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'step_length_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'walking_pattern) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WalkingCommand>) istream)
  "Deserializes a message object of type '<WalkingCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'walk_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'walk_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'walk_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'walk_mode)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'compensator_mode) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'compensator_mode)))
    (cl:dotimes (i 2)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream))))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ik_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ik_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ik_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ik_mode)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heel_toe) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'first_foot_step) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'height) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'step_length_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'step_length_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'walking_pattern) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WalkingCommand>)))
  "Returns string type for a message object of type '<WalkingCommand>"
  "dyros_jet_msgs/WalkingCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WalkingCommand)))
  "Returns string type for a message object of type 'WalkingCommand"
  "dyros_jet_msgs/WalkingCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WalkingCommand>)))
  "Returns md5sum for a message object of type '<WalkingCommand>"
  "7c98f46b805c25b4897e6489a2543122")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WalkingCommand)))
  "Returns md5sum for a message object of type 'WalkingCommand"
  "7c98f46b805c25b4897e6489a2543122")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WalkingCommand>)))
  "Returns full string definition for message of type '<WalkingCommand>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint32 walk_mode~%uint32 NONE = 0~%uint32 STATIC_WALKING = 1~%~%bool[2] compensator_mode~%uint32 HIP_COMPENSTOR = 0~%uint32 EXTERNAL_ENCODER = 1~%~%uint32 ik_mode~%uint32 IK=0~%uint32 JACOBIAN = 1~%uint32 QP = 2~%~%bool heel_toe~%~%bool first_foot_step~%~%float32 x~%float32 y~%float32 z~%float32 height~%float32 theta~%float32 step_length_x~%float32 step_length_y~%~%bool walking_pattern~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WalkingCommand)))
  "Returns full string definition for message of type 'WalkingCommand"
  (cl:format cl:nil "std_msgs/Header header~%~%uint32 walk_mode~%uint32 NONE = 0~%uint32 STATIC_WALKING = 1~%~%bool[2] compensator_mode~%uint32 HIP_COMPENSTOR = 0~%uint32 EXTERNAL_ENCODER = 1~%~%uint32 ik_mode~%uint32 IK=0~%uint32 JACOBIAN = 1~%uint32 QP = 2~%~%bool heel_toe~%~%bool first_foot_step~%~%float32 x~%float32 y~%float32 z~%float32 height~%float32 theta~%float32 step_length_x~%float32 step_length_y~%~%bool walking_pattern~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WalkingCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'compensator_mode) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4
     1
     1
     4
     4
     4
     4
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WalkingCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'WalkingCommand
    (cl:cons ':header (header msg))
    (cl:cons ':walk_mode (walk_mode msg))
    (cl:cons ':compensator_mode (compensator_mode msg))
    (cl:cons ':ik_mode (ik_mode msg))
    (cl:cons ':heel_toe (heel_toe msg))
    (cl:cons ':first_foot_step (first_foot_step msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':height (height msg))
    (cl:cons ':theta (theta msg))
    (cl:cons ':step_length_x (step_length_x msg))
    (cl:cons ':step_length_y (step_length_y msg))
    (cl:cons ':walking_pattern (walking_pattern msg))
))
