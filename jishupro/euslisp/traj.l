;; generate trajectory

;; 2 pos to coords-sequence
(defun traj-p2p (pos0 pos1 split)
  (let* ((coords-seq ())
	 (midp (/ 1.0 (float split))))
    (push (instance coordinates :init :pos pos0) coords-seq)
    (dotimes (i split)
      (push (instance coordinates :init :pos (midpoint (* (+ (float i) 1) midp) pos0 pos1))
	    coords-seq)
      )
    (reverse coords-seq))
  )
  
(defun rarm-ik (target)
  (send *mbot* :rarm :inverse-kinematics
	target
	:link-list (send *mbot* :link-list (send *mbot* :rarm :end-coords :parent))
	:move-target (send *mbot* :rarm :end-coords)
	:rotation-axis nil
	))

(defun mbot-ik (rtarget ltarget)
  (send *mbot* :inverse-kinematics
	(list rtarget ltarget)
	:link-list
	(list (send *mbot* :link-list (send *mbot* :rarm :end-coords :parent))
	      (send *mbot* :link-list (send *mbot* :larm :end-coords :parent)))
	:move-target
	(list (send *mbot* :rarm :end-coords)
	      (send *mbot* :larm :end-coords))
	:rotation-axis (list nil nil)
	))

;; function for wheels
;; flength: forward length, return angle[deg]
(defun l2d (flength)
  (rad2deg (/ flength 26.0)))

(defun walk-test (&optional drawtime)
  (send *mbot* :init-pose)
  (send *mbot* :orient (deg2rad 10) :y :world)
  (let* ((rarm-x (elt (send *mbot* :rarm :end-coords :worldpos) 0))
	 (rarm-y (elt (send *mbot* :rarm :end-coords :worldpos) 1))
	 (split 2)
	 (rarm-wstart (float-vector (+ rarm-x 80) rarm-y 0))
	 (rarm-wend (float-vector (- rarm-x 150) rarm-y 0))
	 (walk-coords-seq (traj-p2p rarm-wstart rarm-wend split))
	 (wrstart-vec (send (cadr walk-coords-seq) :worldpos))
	 (wrend-vec (send (cadr (reverse (cdr walk-coords-seq))) :worldpos))
	 (rarm-wrstart (float-vector (elt wrstart-vec 0) (elt wrstart-vec 1) 80.0))
	 (rarm-wrend (float-vector (elt wrend-vec 0) (elt wrstart-vec 1) 80.0))
	 (walkr-coords-seq (traj-p2p rarm-wrend rarm-wrstart (- split 2)))
	 (walk0-avs ()) ;; slide
	 (walk1-avs ()) ;; return 
	 )
    (dotimes (i (length walk-coords-seq))
      (push (rarm-ik (elt walk-coords-seq i)) walk0-avs)
      (send *irtviewer* :draw-objects)
      (if drawtime (unix:usleep drawtime) (unix:usleep 500000))
      )
    ;; (reverse walk0-avs) ;; slide avs
    (dotimes (i (length walkr-coords-seq))
      (push (rarm-ik (elt walkr-coords-seq i)) walk1-avs)
      (send *irtviewer* :draw-objects)
      (if drawtime (unix:usleep drawtime) (unix:usleep 500000))
      )
    (push (car (reverse walk0-avs)) walk1-avs)
    (send *mbot* :angle-vector (car walk1-avs))
    (send *irtviewer* :draw-objects)
    (if drawtime (unix:usleep drawtime) (unix:usleep 500000))
    (list (reverse walk0-avs) (reverse walk1-avs))
    )
  )

(defun walk0-ri ()
  (let* ((walks (walk-test 0))
	 (walk0 (elt walks 0))
	 (walk1 (elt walks 1))
	 )
    (setq wk0 walk0)
    (setq wk1 walk1)
    (mbot2ri-avs walk0 nil 2.0 1 :thre_dist 10)
    ;; (unix:sleep 10)
    ;;(mbot2ri-avs walk1 2.0 0)
    walk0
    )
  )

(defun walk1-ri ()
  (let* ((walks (walk-test 0))
	 (walk0 (elt walks 0))
	 (walk1 (elt walks 1))
	 )
    ;;(mbot2ri-avs walk0 2.0 0)
    ;; (unix:sleep 10)
    (mbot2ri-avs walk1 nil 2.0 1 :thre_dist 10)
    walk1
    )
  )

(defun walk-ri ()
  (let* ((walks (walk-test 0))
	 (walk0 (elt walks 0))
	 (walk1 (elt walks 1))
	 (walkall (append walk0 walk1))
	 )
    ;;(mbot2ri-avs walk0 2.0 0)
    ;; (unix:sleep 10)
    (setq walka walkall)
    (mbot2ri-avs walkall nil 3.0 1 :thre_dist 10)
    walkall
    )
  )

(defun walk-idv () ;; split in walk-test must be 2
  (let* ((walks (walk-test 0))
	 (walk0 (elt walks 0))
	 (walk1 (elt walks 1))
	 (walkall (append walk0 walk1))
	 (time_list (list 0.0 0.6 1.0 1.4 1.7))
	 )
    (mbot2ri-avs walkall nil time_list 1 :thre_dist 10)
    walkall
    ))

;; use walk-prepare instead of init2walk
(defun init2walk ()
  (let* ((wks (walk-test 0))
	 (winitav (elt (elt wks 0) 0))
	 (avs (list (send *mbot* :init-pose) winitav))
	 )
    ;; (format t "~A~%" avs)
    (mbot2ri-avs avs nil 5.0 0)
    avs
    ))

;; use walk-prepare 1 instead of walk2init
(defun walk2init ()
  (let* ((wks (walk-test 0))
	 (winitav (elt (elt wks 0) 0))
	 (avs (list winitav (send *mbot* :init-pose)))
	 )
    (mbot2ri-avs avs nil 5.0 0)
    avs
    ))

(defun walk-prepare (&optional rvs) ;; calc avs. when rvs, return walk->init
  (send *mbot* :init-pose)
  (send *mbot* :orient (deg2rad 10) :y :world)
  (let* ((posr0 (send *mbot* :rarm :end-coords :worldpos))
	 (posl0 (send *mbot* :larm :end-coords :worldpos))
	 (rarm-y (elt (send *mbot* :rarm :end-coords :worldpos) 1))
	 (larm-y (elt (send *mbot* :larm :end-coords :worldpos) 1))
	 (rarm-x (elt (send *mbot* :rarm :end-coords :worldpos) 0))
	 (larm-x (elt (send *mbot* :larm :end-coords :worldpos) 0))
	 (rtargets (traj-p2p posr0 (float-vector rarm-x rarm-y 0) 3))
	 (ltargets (traj-p2p posl0 (float-vector larm-x larm-y 0) 3))
	 (avs ()))
    (dotimes (i 4)
      (push (mbot-ik (elt rtargets i) (elt ltargets i)) avs)
      (send *irtviewer* :draw-objects)
      (unix:usleep 100000)
      )
    (mbot2ri-avs (if rvs avs (reverse avs)) nil 7.0 1 :thre_dist 10)
    (if rvs avs (reverse avs))
    )
  )
  

(defun walk-both ()
  (send *mbot* :init-pose)
  (send *mbot* :orient (deg2rad 10) :y :world)
  (let* ((rarm-y (elt (send *mbot* :rarm :end-coords :worldpos) 1))
	 (larm-y (elt (send *mbot* :larm :end-coords :worldpos) 1))
	 (rarm-x (elt (send *mbot* :rarm :end-coords :worldpos) 0))
	 (larm-x (elt (send *mbot* :larm :end-coords :worldpos) 0))
	 (distA 80)
	 (distC 100)
	 (distZ 100)
	 (rA (instance coordinates :init :pos (float-vector (+ rarm-x distA) rarm-y 0)))
	 (rB (instance coordinates :init :pos (float-vector rarm-x rarm-y 0)))
	 (rC (instance coordinates :init :pos (float-vector (- rarm-x distC) rarm-y 0)))
	 (rD (instance coordinates :init :pos (float-vector rarm-x rarm-y distZ)))
	 (lA (instance coordinates :init :pos (float-vector (+ larm-x distA) larm-y 0)))
	 (lB (instance coordinates :init :pos (float-vector larm-x larm-y 0)))
	 (lC (instance coordinates :init :pos (float-vector (- larm-x distC) larm-y 0)))
	 (lD (instance coordinates :init :pos (float-vector larm-x larm-y distZ)))
	 (rtargets (list rA rB rC rD rA))
	 (ltargets (list lC lD lA lB lC))
	 ;; (rtargets (list rA rB rC rD rA rB rC rD rA))
	 ;; (ltargets (list lC lD lA lB lC lD lA lB lC))
	 (avs ())
	 (wheels (list (float-vector 0.0 0.0)))
	 ;; (avtime (list 0.0 2.0 2.3 2.35 2.4)) ;; when use this, you should change the value
	 )
    (dotimes (i (length rtargets))
      (push (mbot-ik (elt rtargets i) (elt ltargets i)) avs)
      (if (> i 0)
	  (push (v+ (car wheels) (float-vector (l2d 80) (l2d 80))) wheels))
      (send *irtviewer* :draw-objects)
      (unix:usleep 300000)
      )
    (mbot2ri-avs (reverse avs) (reverse wheels) 4.0 1 :thre_dist 5)
    ;; (mbot2ri-avs (reverse avs) nil 5.0 1 :thre_dist 5) ;; no wheels
    avs
    )
  )