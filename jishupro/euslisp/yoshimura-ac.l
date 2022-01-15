#!/usr/bin/env roseus

;; actionlibクライアントのサンプルプログラム

(load "traj.l")

(ros::roseus-add-msgs "jishupro")

(ros::roseus "eus")

(setq cvg 0)

(defun cvg-cb (msg)
  (setq cvg (send msg :data)))

(ros::subscribe "cvg" std_msgs::int32 #'cvg-cb)

(defun yoshimura-client ()
  (let (goal)
    (if (not (boundp '*c*))
	(setq *c* (instance ros::simple-action-client :init
			    "yoshimura" jishupro::YoshimuraAction)))
    (warning-message 2 ";; ~A wait-for-server~%" (unix::getpid))
    (send *c* :wait-for-server)
    (setq goal (instance jishupro::YoshimuraActionGoal :init))
    (send goal :goal :yoshimura_goal 1) ;; 1:hand 2:nade
    (send *c* :send-goal goal)

    (warning-message 2 ";; ~A wait-for-result~%" (unix::getpid))
    (send *c* :wait-for-result)
    (warning-message 2 ";; ~A result -> ~A~%" (unix::getpid) (send (send *c* :get-result) :yoshimura_result))
    (setq hand-goal (send (send *c* :get-result) :yoshimura_result))
    (cond
     ((= hand-goal 1) (rhand))
     ((= hand-goal 2) (lhand)))
    ;; (setq color (send (send *c* :get-result) :yoshimura_result))
    ))

(defun wait-mbot ()
  (while (= cvg 0)
    (ros::spin-once)))

;; (yoshimura-client)
