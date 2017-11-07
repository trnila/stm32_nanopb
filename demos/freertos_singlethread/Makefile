send: send.c ../../libraries/nanopb/pb_common.c ../../libraries/nanopb/pb_encode.c ../../libraries/nanopb/pb_decode.c ./messages/reverse.pb.c
	gcc $^ -I../../libraries/nanopb -o $@

clean:
	rm -f send
