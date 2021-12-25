sample_model.l: sample_model.dae sample_model.yaml
	rosrun euscollada collada2eus sample_model.dae sample_model.yaml sample_model.l

sample_model.dae: sample_model.urdf
	rosrun collada_urdf_jsk_patch urdf_to_collada sample_model.urdf sample_model.dae
