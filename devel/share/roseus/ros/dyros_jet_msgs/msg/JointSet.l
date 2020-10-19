;; Auto-generated. Do not edit!


(when (boundp 'dyros_jet_msgs::JointSet)
  (if (not (find-package "DYROS_JET_MSGS"))
    (make-package "DYROS_JET_MSGS"))
  (shadow 'JointSet (find-package "DYROS_JET_MSGS")))
(unless (find-package "DYROS_JET_MSGS::JOINTSET")
  (make-package "DYROS_JET_MSGS::JOINTSET"))

(in-package "ROS")
;;//! \htmlinclude JointSet.msg.html


(defclass dyros_jet_msgs::JointSet
  :super ros::object
  :slots (_id _angle ))

(defmethod dyros_jet_msgs::JointSet
  (:init
   (&key
    ((:id __id) 0)
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _angle (float __angle))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; uint8 _id
    1
    ;; float64 _angle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _id
       (write-byte _id s)
     ;; float64 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float64 _angle
     (setq _angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get dyros_jet_msgs::JointSet :md5sum-) "9c94549c9e988fc6422291c5c0b1ea5b")
(setf (get dyros_jet_msgs::JointSet :datatype-) "dyros_jet_msgs/JointSet")
(setf (get dyros_jet_msgs::JointSet :definition-)
      "uint8 id
float64 angle


")



(provide :dyros_jet_msgs/JointSet "9c94549c9e988fc6422291c5c0b1ea5b")


