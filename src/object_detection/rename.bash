for i in `ls include/object_detection`
do
	sed 's/OBJECT_DETECTION_3D/OBJECT_DETECTION/g
	s/object_detection_3d/object_detection/g' include/object_detection/$i >include/object_detection/new_$i
	mv include/object_detection/new_$i include/object_detection/$i
done
