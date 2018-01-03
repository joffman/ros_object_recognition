for i in `ls include/binary_detector`
do
	sed 's/BINARY_DETECTOR_NODE/BINARY_DETECTOR/g
	s/binary_detector_node/binary_detector/g' include/binary_detector/$i >include/binary_detector/new_$i
	mv include/binary_detector/new_$i include/binary_detector/$i
done

for i in `ls src`
do
	sed 's/BINARY_DETECTOR_NODE/BINARY_DETECTOR/g
	s/binary_detector_node/binary_detector/g' src/$i >src/new_$i
	mv src/new_$i src/$i
done
