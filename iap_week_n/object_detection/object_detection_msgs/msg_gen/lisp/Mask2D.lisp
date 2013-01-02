; Auto-generated. Do not edit!


(cl:in-package object_detection_msgs-msg)


;//! \htmlinclude Mask2D.msg.html

(cl:defclass <Mask2D> (roslisp-msg-protocol:ros-message)
  ((height
    :reader height
    :initarg :height
    :type cl:integer
    :initform 0)
   (width
    :reader width
    :initarg :width
    :type cl:integer
    :initform 0)
   (isValid
    :reader isValid
    :initarg :isValid
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 0 :element-type 'cl:boolean :initial-element cl:nil)))
)

(cl:defclass Mask2D (<Mask2D>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Mask2D>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Mask2D)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name object_detection_msgs-msg:<Mask2D> is deprecated: use object_detection_msgs-msg:Mask2D instead.")))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <Mask2D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detection_msgs-msg:height-val is deprecated.  Use object_detection_msgs-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <Mask2D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detection_msgs-msg:width-val is deprecated.  Use object_detection_msgs-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'isValid-val :lambda-list '(m))
(cl:defmethod isValid-val ((m <Mask2D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader object_detection_msgs-msg:isValid-val is deprecated.  Use object_detection_msgs-msg:isValid instead.")
  (isValid m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Mask2D>) ostream)
  "Serializes a message object of type '<Mask2D>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'isValid))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'isValid))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Mask2D>) istream)
  "Deserializes a message object of type '<Mask2D>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'isValid) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'isValid)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Mask2D>)))
  "Returns string type for a message object of type '<Mask2D>"
  "object_detection_msgs/Mask2D")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Mask2D)))
  "Returns string type for a message object of type 'Mask2D"
  "object_detection_msgs/Mask2D")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Mask2D>)))
  "Returns md5sum for a message object of type '<Mask2D>"
  "c073105a22d894f799b67a16c88b190a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Mask2D)))
  "Returns md5sum for a message object of type 'Mask2D"
  "c073105a22d894f799b67a16c88b190a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Mask2D>)))
  "Returns full string definition for message of type '<Mask2D>"
  (cl:format cl:nil "uint32 height~%uint32 width~%bool[] isValid~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Mask2D)))
  "Returns full string definition for message of type 'Mask2D"
  (cl:format cl:nil "uint32 height~%uint32 width~%bool[] isValid~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Mask2D>))
  (cl:+ 0
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'isValid) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Mask2D>))
  "Converts a ROS message object to a list"
  (cl:list 'Mask2D
    (cl:cons ':height (height msg))
    (cl:cons ':width (width msg))
    (cl:cons ':isValid (isValid msg))
))
