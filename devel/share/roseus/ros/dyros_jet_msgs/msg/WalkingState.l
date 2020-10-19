;; Auto-generated. Do not edit!


(when (boundp 'dyros_jet_msgs::WalkingState)
  (if (not (find-package "DYROS_JET_MSGS"))
    (make-package "DYROS_JET_MSGS"))
  (shadow 'WalkingState (find-package "DYROS_JET_MSGS")))
(unless (find-package "DYROS_JET_MSGS::WALKINGSTATE")
  (make-package "DYROS_JET_MSGS::WALKINGSTATE"))

(in-package "ROS")
;;//! \htmlinclude WalkingState.msg.html


(defclass dyros_jet_msgs::WalkingState
  :super ros::object
  :slots (_walking_end _walking_end_foot_side ))

(defmethod dyros_jet_msgs::WalkingState
  (:init
   (&key
    ((:walking_end __walking_end) nil)
    ((:walking_end_foot_side __walking_end_foot_side) nil)
    )
   (send-super :init)
   (setq _walking_end __walking_end)
   (setq _walking_end_foot_side __walking_end_foot_side)
   self)
  (:walking_end
   (&optional __walking_end)
   (if __walking_end (setq _walking_end __walking_end)) _walking_end)
  (:walking_end_foot_side
   (&optional __walking_end_foot_side)
   (if __walking_end_foot_side (setq _walking_end_foot_side __walking_end_foot_side)) _walking_end_foot_side)
  (:serialization-length
   ()
   (+
    ;; bool _walking_end
    1
    ;; bool _walking_end_foot_side
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _walking_end
       (if _walking_end (write-byte -1 s) (write-byte 0 s))
     ;; bool _walking_end_foot_side
       (if _walking_end_foot_side (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _walking_end
     (setq _walking_end (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _walking_end_foot_side
     (setq _walking_end_foot_side (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get dyros_jet_msgs::WalkingState :md5sum-) "e5b99df3c7eac3f1f42ddf9f1822ced4")
(setf (get dyros_jet_msgs::WalkingState :datatype-) "dyros_jet_msgs/WalkingState")
(setf (get dyros_jet_msgs::WalkingState :definition-)
      "bool walking_end
bool walking_end_foot_side


")



(provide :dyros_jet_msgs/WalkingState "e5b99df3c7eac3f1f42ddf9f1822ced4")


