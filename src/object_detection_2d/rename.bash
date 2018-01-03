for i in `ls include/object_detection_2d`
do
	sed 's/OBJECT_DETECTION_2D_NODES/OBJECT_DETECTION_2D/g
	s/object_detection_2d_nodes/object_detection_2d/g' include/object_detection_2d/$i >include/object_detection_2d/new_$i
	mv include/object_detection_2d/new_$i include/object_detection_2d/$i
done

for i in `ls src`
do
	sed 's/OBJECT_DETECTION_2D_NODES/OBJECT_DETECTION_2D/g
	s/object_detection_2d_nodes/object_detection_2d/g' src/$i >src/new_$i
	mv src/new_$i src/$i
done
