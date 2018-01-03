for i in `ls include/object_detection_3d`
do
	sed 's/OBJECT_DETECTION_3D_NODES/OBJECT_DETECTION_3D/g
	s/object_detection_3d_nodes/object_detection_3d/g' include/object_detection_3d/$i >include/object_detection_3d/new_$i
	mv include/object_detection_3d/new_$i include/object_detection_3d/$i
done
