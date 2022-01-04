(Load "../models/mbot.l")
(mbot)
(send *mbot* :init-pose)
;; (send *mbot* :wire-init)

;; (draw-wire-end)

(setq wire-list (float-vector 0 0 0 0 0 0 0 0 0 0))

;; (p, q) : vector from joint to a point
;; r : radius
(defun calc-wire (p q r)
  (let* ((l (sqrt (+ (* p p) (* q q) (- (* r r)))))
	 (k (/ r (+ (* p p) (* q q))))
	 (x0 (* k (+ (* p r) (* q l))))
	 (y0 (* k (- (* q r) (* p l))))
	 (x1 (* k (- (* p r) (* q l))))
	 (y1 (* k (+ (* q r) (* p l)))))
    (list x0 y0 x1 y1)))

(defmethod mbot-robot
  ;; 0-4:right 5-9:left
  ;; 0: elbow-front
  ;; 1: elbow-back
  ;; 2: shoulder-front
  ;; 3: shoulder-back
  ;; 4: shoulder
  (:wire-init
   ()
;;   (send self :zero-pose)
   
   (setq relbow-front-start (wire-assoc (send self :rleg_lk) 14.75 0 -136.5))   
   (setq relbow-back-start (wire-assoc (send self :rleg_lk) -14.75 0 -136.5))
   (setq relbow-front-end (wire-assoc (send self :rfoot_lk) 9.75 0 -69.7))
   (setq relbow-back-end (wire-assoc (send self :rfoot_lk) -19.75 0 -69.7))
   (setq relbow-joint (wire-assoc (send self :rfoot_lk) 0 0 0))
   
   (setq lelbow-front-start (wire-assoc (send self :lleg_lk) 14.75 0 -136.5))   
   (setq lelbow-back-start (wire-assoc (send self :lleg_lk) -14.75 0 -136.5))
   (setq lelbow-front-end (wire-assoc (send self :lfoot_lk) 9.75 0 -69.7))
   (setq lelbow-back-end (wire-assoc (send self :lfoot_lk) -19.75 0 -69.7))
   (setq lelbow-joint (wire-assoc (send self :lfoot_lk) 0 0 0))

   (setq ref-start (wire-assoc (send self :root_lk) 217.25 -102 428.4))
   (setq rs-start (wire-assoc (send self :root_lk) 207.5 -102 428.4))
   (setq reb-start (wire-assoc (send self :root_lk) 197.75 -102 428.4))
   (setq rsb-start (wire-assoc (send self :root_lk) 157.5 -77.5 399.4))
   (setq rsf-start (wire-assoc (send self :root_lk) 257.5 -77.5 399.4))

   (setq ref-end (wire-assoc (send self :rleg_lk) 13 -16 38.64))
   (setq rs-end (wire-assoc (send self :rleg_lk) 0 -16 39.8))
   (setq reb-end (wire-assoc (send self :rleg_lk) -13 -16 38.64))
   (setq rsf-end (wire-assoc (send self :rleg_lk) 10 10 -58.5))
   (setq rsb-end (wire-assoc (send self :rleg_lk) -10 10 -58.5))

   (setq lef-start (wire-assoc (send self :root_lk) 217.25 102 428.4))
   (setq ls-start (wire-assoc (send self :root_lk) 207.5 102 428.4))
   (setq leb-start (wire-assoc (send self :root_lk) 197.75 102 428.4))
   (setq lsb-start (wire-assoc (send self :root_lk) 157.5 77.5 399.4))
   (setq lsf-start (wire-assoc (send self :root_lk) 257.5 77.5 399.4))

   (setq lef-end (wire-assoc (send self :lleg_lk) 13 16 38.64))
   (setq ls-end (wire-assoc (send self :lleg_lk) 0 16 39.8))
   (setq leb-end (wire-assoc (send self :lleg_lk) -13 16 38.64))
   (setq lsf-end (wire-assoc (send self :lleg_lk) 10 -10 -58.5))
   (setq lsb-end (wire-assoc (send self :lleg_lk) -10 -10 -58.5))
   
   t)
  (:wire-calc
   ()
   (setf (elt wire-list 0) (dcasc ref-start ref-end))
   (setf (elt wire-list 1) (dcasc reb-start reb-end))
   (setf (elt wire-list 2) (dcasc rsf-start rsf-end))
   (setf (elt wire-list 3) (dcasc rsb-start rsb-end))
   (setf (elt wire-list 4) (dcasc rs-start rs-end))
   (setf (elt wire-list 5) (dcasc lef-start lef-end))
   (setf (elt wire-list 6) (dcasc leb-start leb-end))
   (setf (elt wire-list 7) (dcasc lsf-start lsf-end))
   (setf (elt wire-list 8) (dcasc lsb-start lsb-end))
   (setf (elt wire-list 9) (dcasc ls-start ls-end))
   (let* ((rbs (wire-calc-elbow relbow-back-start relbow-joint 31 1))
	  (rbd (elt rbs 0))
	  (rbc (elt rbs 1))
	  (rbe (wire-calc-elbow relbow-back-end relbow-joint 31 -1))
	  (rba (elt rbe 0))
	  (rbb (elt rbe 1))
	  (rb-len (wire-calc-length-elbow rba rbb rbc rbd 31))

	  (rfs (wire-calc-elbow relbow-front-start relbow-joint 31 -1))
	  (rfa (elt rfs 0))
	  (rfb (elt rfs 1))
	  (rfe (wire-calc-elbow relbow-front-end relbow-joint 31 1))
	  (rfd (elt rfe 0))
	  (rfc (elt rfe 1))
	  (rf-len (wire-calc-length-elbow rfa rfb rfc rfd 31))
	  
	  (lbs (wire-calc-elbow lelbow-back-start lelbow-joint 31 1))
	  (lbd (elt lbs 0))
	  (lbc (elt lbs 1))
	  (lbe (wire-calc-elbow lelbow-back-end lelbow-joint 31 -1))
	  (lba (elt lbe 0))
	  (lbb (elt lbe 1))
	  (lb-len (wire-calc-length-elbow lba lbb lbc lbd 31))

	  (lfs (wire-calc-elbow lelbow-front-start lelbow-joint 31 -1))
	  (lfa (elt lfs 0))
	  (lfb (elt lfs 1))
	  (lfe (wire-calc-elbow lelbow-front-end lelbow-joint 31 1))
	  (lfd (elt lfe 0))
	  (lfc (elt lfe 1))
	  (lf-len (wire-calc-length-elbow lfa lfb lfc lfd 31))
	  )
     (setf (elt wire-list 0) (+ (elt wire-list 0) rf-len))
     (setf (elt wire-list 1) (+ (elt wire-list 1) rb-len))
     (setf (elt wire-list 5) (+ (elt wire-list 5) lf-len))
     (setf (elt wire-list 6) (+ (elt wire-list 6) lb-len))
     )
   wire-list
   )
  )

(defun dcasc (casc0 casc1)
  (distance (send casc0 :worldpos) (send casc1 :worldpos)))

(defun wire-calc-elbow (wpoint jt r is_plus)
  (let* ((vec0 (v- (send jt :inverse-transform-vector (send wpoint :worldpos))))
	 (theta-r (* is_plus (asin (/ r (norm vec0)))))
	 (vec-pr (rotate-vector vec0 theta-r :y))
	 (vec-proj (scale (cos theta-r) vec-pr))
	 (vec-r (v- vec-proj vec0))
	 )
    (list (v- vec0) vec-r)))

(defun wire-calc-length-elbow (a b c d r)
  (let* ((ab (distance a b))
	 (ccd (distance c d))
	 (ac (distance a c))
	 (bd (distance b d))
	 (wlen (distance a d))
	 ;; (wlen_theta_tmp (asin (/ (elt (v* b c) 1) (* (norm b) (norm c)))))
	 ;; (wlen_theta (if (< wlen_theta_tmp 0) (+ wlen_theta_tmp 2pi) wlen_theta_tmp))
	 (wlen_theta_tmp (acos (/ (v. b c) (* (norm b) (norm c)))))
	 (wlen_theta_cp (elt (v* b c) 1))
	 (wlen_theta (if (< wlen_theta_cp 0) (+ wlen_theta_tmp pi) wlen_theta_tmp))
	 (wlen_arc (* r wlen_theta))
	 )
    (if (< (+ ab ccd) (+ ac bd)) ;; ワイヤーが接触
	(+ ab ccd wlen_arc)
      (distance a d))
    )
  )

(defun wire-calc-length-elbow-q (a b c d r)
  (setq aaa a)
  (setq bbb b)
  (setq ccc c)
  (setq ddd d)
  (setq ab (distance a b))
  (setq ccd (distance c d))
  (setq ac (distance a c))
  (setq bd (distance b d))
  (setq wlen (distance a d))
  (setq wlen_theta_tmp (acos (/ (v. b c) (* (norm b) (norm c)))))
  (setq wlen_theta_cp (elt (v* b c) 1))
  (setq wlen_theta (if (< wlen_theta_cp 0) (+ wlen_theta_tmp pi) wlen_theta_tmp))
  (setq wlen_arc (* r wlen_theta))
  (setq wlen_ans (if (< (+ ab ccd) (+ ac bd)) ;; ワイヤーが接触
		     (+ ab ccd (* r wlen_theta))
		   (distance a d)))
  )

(defun wire-assoc (bl x y z)
  (let* ((wire-point (instance cascaded-coords :init
			       :coords (send bl :copy-worldcoords)))
	 )
    (send wire-point :locate (float-vector x y z) :local)
    (send bl :assoc wire-point)
    wire-point))

(defun draw-wire-end ()
  (objects (list *mbot*
		 relbow-front-start
		 relbow-back-start
		 relbow-front-end
		 relbow-back-end
		 relbow-joint
		 ref-start
		 rs-start
		 reb-start
		 rsb-start
		 rsf-start
		 ref-end
		 rs-end
		 reb-end
		 rsb-end
		 rsf-end
		 lef-start
		 lef-end
		 ls-start
		 ls-end
		 leb-start
		 leb-end
		 lsb-start
		 lsb-end
		 lsf-start
		 lsf-end
		 ;relbow-c
		 ;relbow-d
		 ;relbow-e
		 ;relbow-f
		 ))
  )
