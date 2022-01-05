#!/usr/bin/env roseus
;;
(when (not (boundp '*mbot*))
  (load "mbot-utils.l")
  (mbot)
k  (send *mbot* :wire-init)
  (send *mbot* :init-pose)
  )

;; (send *mbot* :wire-init)
;; initialize wire
;; (send *mbot* :init-pose)
;; (send *mbot* :open-pose)

(setq wire-list-arduino (float-vector 0 0 0 0 0 0 0 0 0 0))
(setq wire-list-arduino-idx (list 2 0 4 1 3 7 5 9 6 8))

(defun wire-list-to-arduino ()
  (dotimes (i 10)
    (setf (elt wire-list-arduino i) (elt wire-list (elt wire-list-arduino-idx i)))))

(ros::roseus-add-msgs "std_msgs")

(defun mbot2ri ()
  (ros::roseus "eus")
  (ros::advertise "wireus" std_msgs::float32multiarray 1)
  (ros::rate 10)
  ;;(do-until-key
  ;; (while (ros::ok)
  (dotimes (i 50)
    (setq msg (instance std_msgs::float32multiarray :init))
    (send *mbot* :wire-calc)
    (wire-list-to-arduino)
    (send msg :data wire-list-arduino)
    (ros::publish "wireus" msg)
    (ros::sleep))
  )
