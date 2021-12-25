mbot.l: mbot.dae mbot.yaml
	rosrun euscollada collada2eus mbot.dae mbot.yaml mbot.l

mbot.dae: mbot.urdf
	rosrun collada_urdf_jsk_patch urdf_to_collada mbot.urdf mbot.dae
