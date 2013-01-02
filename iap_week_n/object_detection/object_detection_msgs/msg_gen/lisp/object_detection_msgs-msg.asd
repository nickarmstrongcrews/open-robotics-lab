
(cl:in-package :asdf)

(defsystem "object_detection_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Mask2D" :depends-on ("_package_Mask2D"))
    (:file "_package_Mask2D" :depends-on ("_package"))
  ))