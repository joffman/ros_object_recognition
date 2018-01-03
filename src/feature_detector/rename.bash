for i in `ls include/feature_detector`
do
	sed 's/FEATURE_DETECTOR_NODE/FEATURE_DETECTOR/g
	s/feature_detector_node/feature_detector/g' include/feature_detector/$i >include/feature_detector/new_$i
	mv include/feature_detector/new_$i include/feature_detector/$i
done

for i in `ls src`
do
	sed 's/FEATURE_DETECTOR_NODE/FEATURE_DETECTOR/g
	s/feature_detector_node/feature_detector/g' src/$i >src/new_$i
	mv src/new_$i src/$i
done
