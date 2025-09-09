# Video quick start
RX (receiver): `./video/gst_rx.sh 5004`
TX (sender, camera): `./video/gst_tx.sh 127.0.0.1 5004`
TX (file): `VIDEO_FILE=sample.mp4 ./video/gst_tx.sh 127.0.0.1 5004`
Tune bitrate: `BITRATE=3000 ...`
