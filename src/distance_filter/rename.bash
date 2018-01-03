for i in `ls include/distance_filter`
do
	sed 's/DISTANCE_FILTER_NODE/DISTANCE_FILTER/g
	s/distance_filter_node/distance_filter/g' include/distance_filter/$i >include/distance_filter/new_$i
	mv include/distance_filter/new_$i include/distance_filter/$i
done

for i in `ls src`
do
	sed 's/DISTANCE_FILTER_NODE/DISTANCE_FILTER/g
	s/distance_filter_node/distance_filter/g' src/$i >src/new_$i
	mv src/new_$i src/$i
done
