/*
Disclaimer - intended for development only.  Not recommended for competition or official events
Using the port forwarding for configuration or streaming to dashboard may use significant
bandwidth and cause problems with robots in a dense competition setting.

We do think that the setDriverStationStreamSource() method utilizes minimal bandwidth, sending
single snapshots as intended by the drivers station app and FTC SDK. Its use may be appropriate
at event venues.

Following is from any email with Danny Diaz, FTC Senior Engineering Manager
"With that in mind, we do have an important request regarding the use of this particular tool at
official FTC events. Due to the very limited and critical nature of Wi-Fi bandwidth at our events,
tools that place a significant burden on the network can unfortunately impact the overall event
experience for everyone. Therefore, we would greatly appreciate it if you could prominently
include a statement in your repository explicitly advising users that this tool should absolutely
not be used at official FTC events. This includes the competition area, pit area, and practice
fields. It's crucial to emphasize that using such a tool at an event is forbidden due to the
potential strain on the event's network infrastructure. This is a policy we maintain across the
board, and it's the same reason we don't allow streaming to the Driver Station."
*/
package org.firstinspires.ftc.teamcode.sensors.limelight;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.lang.reflect.Field;
import java.net.HttpURLConnection;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.URL;
import java.util.Arrays;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Tools to make LimeLight easier to use by FTC teams.  Grab images, stream video or use
 * web interface while connected to Robot Controller.
 */
public class LimeLightImageTools {
    Limelight3A limeLight;
    private String ipAddress = "0.0.0.0";

    public LimeLightCameraStreamSource streamSource;



    public static class LimeLightCameraStreamSource implements CameraStreamSource {

        private String ipAddress;

        public LimeLightCameraStreamSource(String ipAddr) {
            ipAddress = ipAddr;
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            // Convert BufferedImage to Bitmap and pass it to the consumer
            LimeLightImageTools llIt = new LimeLightImageTools(ipAddress);
            Bitmap bmpBitmap = llIt.getProcessedBMP();
            continuation.dispatch(bmpBitmapConsumer -> bmpBitmapConsumer.accept(bmpBitmap));

        }
    }


    /**
     *
     * @param limeLight  sensor to be accessed by this class
     */
    public LimeLightImageTools(Limelight3A limeLight) {
        this.limeLight = limeLight;
        try {
            Field privatIPaddress = Limelight3A.class.getDeclaredField("inetAddress");
            privatIPaddress.setAccessible(true);
            InetAddress ipAddr = (InetAddress) privatIPaddress.get(limeLight);
            assert ipAddr != null;
            this.ipAddress = ipAddr.getHostAddress();
        } catch (Exception e) {
            RobotLog.d("LLIT Failed to get IP address" );
        }
        streamSource = new LimeLightCameraStreamSource(ipAddress);
    }

    /**
     * An alternate consturor that uses a provided IP address rather than getting it from the limelight.
     * This should work without needed dependencies on the FTC sdk for this library
     * @param ipAddr  String containing the Ip address of the limelight.  Typically "172.29.0.1" verify in robot config
     */
    public LimeLightImageTools(String ipAddr) {

        this.ipAddress = ipAddr;
        streamSource = new LimeLightCameraStreamSource(ipAddress);

    }

    // ***  Begin access images like the webpage does when PC plugged into camera  ***

    /**
     * get a Bitmap from the limelight of the raw (unprocessed)
     * @return the raw image
     */
    public Bitmap getRawBMP() {
        return getMultiPartBMP(":5802");
    }

    /**
     * get a Bitmap from the limelight of the processed image
     * @return the processed image
     */
    public Bitmap getProcessedBMP() {
        return getMultiPartBMP(":5800");
    }

    /**
     * get an image from the stream coming from the limelight.
     * image is deconded from a MultipartWriter with multipart/x-mixed-replace stream.
     * This is intended for single image grabs, but can be used for slow streaming, but
     * it may be a little slow.
     * @param port The port number to access the stream, 5800=processed image, 5802=raw image
     * @return the requested image as a Bitmap
     */
    public  Bitmap getMultiPartBMP(String port)  {
        HttpURLConnection connection = openConnection("http://" + ipAddress + port);
        try {
            if (connection != null) {
                InputStream inputStream = connection.getInputStream();
                String contentType = connection.getHeaderField("Content-Type");
                String boundary = extractBoundary(contentType);
                if (boundary == null) {
                    RobotLog.d("LLIT boundary is NULL Exception - ");
                    return null;
                }
                return readImageFromStream(inputStream, boundary);
            }
        } catch (Exception e) {
            RobotLog.d("LLIT decodeMultipartImage Exception - " + e );
            return null;
        } finally {
            if (connection != null) {
                connection.disconnect();
            }
        }
        return null;
    }

    /**
     * Open a URL connection
     * @param urlString the ipaddress and port to be opened
     * @return the connection
     */
    private HttpURLConnection openConnection(String urlString) {
        int GETREQUEST_TIMEOUT = 100;
        int CONNECTION_TIMEOUT = 100;

        try {
            URL url = new URL(urlString);
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("GET");
            connection.setReadTimeout(GETREQUEST_TIMEOUT);
            connection.setConnectTimeout(CONNECTION_TIMEOUT);
            return connection;
        } catch (Exception e) {
            RobotLog.d("LLIT openConnection Exception - " + e );
        }
        return null;
    }

    /**
     * Find the boundary string
     * Example:   "multipart/x-mixed-replace;boundary=boundarydonotcross"
     *            would return "boundarydonotcross"
     * @param contentType the header field to search
     * @return The boundary string
     */
    private  String extractBoundary(String contentType) {
        Pattern pattern = Pattern.compile("boundary=(.*)");
        Matcher matcher = pattern.matcher(contentType);
        if (matcher.find()) {
            return matcher.group(1);
        }
        return null;
    }

    /**
     * Extract Bitmap from an image stream that is encoded with MultipartWriter with multipart/x-mixed-replace.
     * @param inputStream The stream to parse
     * @param boundary The boundary of each item
     * @return our desired Bitmap or null if not found.
     */
    private Bitmap readImageFromStream(InputStream inputStream , String boundary) {
        String marker = "--" + boundary;
        byte[] buffer = new byte[1000];
        int bytesRead=0;
        StringBuilder data = new StringBuilder();
        Bitmap bmp = null;

        try {
            if (inputStream != null) {
                bytesRead = inputStream.read(buffer);
                data.append(new String(buffer, 0, bytesRead));
                String dataString = data.toString();
                String LengthStringKey = "Content-Length: ";
                int sizeIndexStart = dataString.indexOf(LengthStringKey) + LengthStringKey.length();
                int sizeIndexEnd = dataString.indexOf("\r\n", sizeIndexStart);
                String sizeString = dataString.substring(sizeIndexStart, sizeIndexEnd);
                int size = Integer.parseInt(sizeString);
                int imageStart = dataString.indexOf("\r\n\r\n")+4;
                byte[] byteBuffer = Arrays.copyOfRange(buffer, imageStart, size+1000); // size is used to force new buffer size
                bytesRead -= imageStart; // remove the header bytes and start counting at image start.

                int i = 0;
                while (bytesRead < size && i != -1) {
                    int numberToRead = size-bytesRead;
                    i = inputStream.read(byteBuffer, bytesRead, numberToRead);
                    bytesRead += i;
                }
                if (i==-1) {
                    RobotLog.d("LLIT no more bytes to get" );
                }

                bmp = BitmapFactory.decodeStream(new ByteArrayInputStream(byteBuffer));
                if (bmp==null) {
                    RobotLog.d("LLIT bmp is null" );
                }
                return bmp;

            } else {
                RobotLog.d("LLIT readImageFromStream - input stream null" );
                return bmp;
            }
        } catch (Exception e) {
            RobotLog.d("LLIT readImageFromStream Exception byteRead="+bytesRead+"  e=" + e);
            //e.printStackTrace();
            return null;
        }
    }
    // ***  End access pictures like the webpage does when plugged into camera  ***




    // ***  Start try to do port forwarding through so can configure limelight through control hub
    // TODO
    //   - Need a propery way to shut down threads, or do they terminate and get cleaned up.
    //   - revisit if and when we need to clientSocket.close()
    //   - and finalClientSocket is not optimal solutions, came from compiler's suggestion.
    //   - look at outputclose in startForwarding()

    /**
     * Forward a port from the Robot Controller to the LimeLight.
     * @param port  The port number to forward.  Same port is used on Robot Controller and Limelight
     */
    public void portForwarding(int port) {
        int localPort = port; // Port to listen on
        String remoteHost = ipAddress; //"remote_host"; // Host to forward to
        int remotePort = port; // Port on remote host to forward to

        try {
            ServerSocket serverSocket = new ServerSocket(localPort);
            System.out.println("Listening on port " + localPort);
            RobotLog.d("LLIT portForwarding Listening on port " + localPort);


            Thread listenThread = new Thread(() ->{  // this thread continously listens for new connections to the server.

                while (true) {
                    RobotLog.d("LLIT portForwarding " + port + " waiting for client on " + localPort);
                    Socket clientSocket = null;
                    try {
                        clientSocket = serverSocket.accept();   // waits for client to request the port
                    } catch (IOException e) {
                        RobotLog.d("LLIT portForwarding " + port + " no client Socket");
                    }
                    System.out.println("Accepted connection from " + clientSocket.getInetAddress());
                    RobotLog.d("LLIT portForwarding " + port + " Accepted connection from " + clientSocket.getInetAddress());

                    Socket finalClientSocket = clientSocket;
                    Thread forwardThread = new Thread(() -> {
                        try {
                            Socket remoteSocket = new Socket(remoteHost, remotePort);
                            System.out.println("Connected to remote host " + remoteHost + ":" + remotePort);
                            RobotLog.d("LLIT portForwarding " + port + " Connected to remote host " + remoteHost + ":" + remotePort);

                            // Start threads to forward data in both directions
                            startForwarding(finalClientSocket.getInputStream(), remoteSocket.getOutputStream(), "client:"+port);  // starts thread for client to remote
                            startForwarding(remoteSocket.getInputStream(), finalClientSocket.getOutputStream(), "host:"+port);  // starts thread for remote to client

                        } catch (IOException e) {
                            RobotLog.d("LLIT portForwarding " + port + " Error forwarding: " + e.getMessage());

                        } finally {
//                        try {
//                            finalClientSocket.close();
//                            RobotLog.d("LLIT portForwarding client Socket Closed");
//                        } catch (IOException e) {
//                            // Ignore
//                        }
                        }
                    });
                    forwardThread.start();
                }
            });
            listenThread.start();
        } catch (IOException e) {
            RobotLog.d("LLIT portForwarding " + port + " Error starting server: " + e.getMessage());
        }
    }

    /**
     * Start a new thread to forward a port as long as there are bytes to forward.
     * forward input to output only, 1 direction only!
     * @param input  Incoming Stream
     * @param output Destination Stream
     * @param name a nickname for this forwad, using for logging.
     */
    private static void startForwarding(InputStream input, OutputStream output, String name) {
        Thread thread = new Thread(() -> {
            byte[] buffer = new byte[4096];
            int bytesRead;
            try {
                while ((bytesRead = input.read(buffer)) != -1) {
                    output.write(buffer, 0, bytesRead);
                    output.flush();
                }
            } catch (IOException e) {
                // Connection probably closed
            } finally {
                try {
                    output.close();
                    RobotLog.d("LLIT startForwarding closed, "+name);
                } catch (IOException e) {
                    // Ignore
                }
            }
        });
        thread.start();
    }


    /**
     * Begins port forwarding for the required ports to enable accessing the LimeLight through
     * the Robot Controller.
     */
    public void forwardAll() {
        portForwarding(5800);
        portForwarding(5801);
        portForwarding(5802);
        portForwarding(5805);
    }

    /**
     * Begins port forwarding for the port to enable streaming of the Prcoessed image.
     *
     */
    public void forwardProcessedStream(){
        portForwarding(5800);
    }


    // ***  end try to do port forwarding through so can configure limelight through control hub

    /**
     * use the Limewire as a stream source for the driver station's "camera stream" during Init.
     */
    public void setDriverStationStreamSource() {
        CameraStreamServer.getInstance().setSource(streamSource);
    }

    /**
     * Get the stream Source to pass it to dashboard or other consumers.
     * The stream is opened, one frame captured, and then closed.
     * @return the limeLight's streaming source.
     */
    public LimeLightCameraStreamSource getStreamSource () {
        return streamSource;
    }


}