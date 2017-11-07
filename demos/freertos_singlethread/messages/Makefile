all: reverse.pb.c

%.pb.c: %.pb
	python ../../../libraries/nanopb/generator/nanopb_generator.py reverse.pb

%.pb: %.proto
	protoc $^ -o $@

clean:
	rm -f *.pb *.pb.c *.pb.h
