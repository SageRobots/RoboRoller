var http = require('http');
var fs = require('fs');
var btSerial = new (require("bluetooth-serial-port").BluetoothSerialPort)();
var connected = false;

function btWrite(data) {
   	console.log("writing: " + String(data));
    btSerial.write(
        Buffer.from(String(data), "utf-8"),
        function (err, bytesWritten) {
            if (err) console.log(err);
        }
    );
};

http.createServer(function (req, res) {
	if(req.url == "/") {
		fs.readFile("./index.html", function(err, data) {
			res.writeHead(200, {'Content-Type': 'text/html'});
			res.write(data);
			return res.end();
		});
	} else if (req.url == "/connect") {
		console.log(req.url);
		btSerial.inquire();
	} else if (req.url == "/status") {
		res.writeHead(200);
		if(connected) {
			res.write("connected,1");
		} else {
			res.write("connected,0");
		}
		res.end();
	} else if (req.url.substr(0,6) == "/movex") {
		btWrite(req.url.substr(1));
	} else {
		console.log(req.url);
	}
}).listen(8080);

btSerial.on("found", function (address, name) {
    btSerial.findSerialPortChannel(
        address,
        function (channel) {
            btSerial.connect(
                address,
                channel,
                function () {
                    console.log("connected");
                    connected = true;
                    btSerial.write(
                        Buffer.from("my data\n", "utf-8"),
                        function (err, bytesWritten) {
                            if (err) console.log(err);
                        }
                    );

                    btSerial.on("data", function (buffer) {
                        console.log(buffer.toString("utf-8"));
                    });
                },
                function () {
                    console.log("cannot connect");
                }
            );

            // close the connection when you're ready
            // btSerial.close();
        },
        function () {
            console.log("found nothing");
        }
    );
});

// btSerial.inquire();