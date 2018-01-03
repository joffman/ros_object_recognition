for i in `ls include/morphology_filter`
do
	sed 's/MORPHOLOGY_FILTER_NODE/MORPHOLOGY_FILTER/g
	s/morphology_filter_node/morphology_filter/g' include/morphology_filter/$i >include/morphology_filter/new_$i
	mv include/morphology_filter/new_$i include/morphology_filter/$i
done

for i in `ls src`
do
	sed 's/MORPHOLOGY_FILTER_NODE/MORPHOLOGY_FILTER/g
	s/morphology_filter_node/morphology_filter/g' src/$i >src/new_$i
	mv src/new_$i src/$i
done
