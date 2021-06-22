
(cl:in-package :asdf)

(defsystem "rt_dynamixel_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Error" :depends-on ("_package_Error"))
    (:file "_package_Error" :depends-on ("_package"))
    (:file "JointSet" :depends-on ("_package_JointSet"))
    (:file "_package_JointSet" :depends-on ("_package"))
    (:file "JointState" :depends-on ("_package_JointState"))
    (:file "_package_JointState" :depends-on ("_package"))
  ))