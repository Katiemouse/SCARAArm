
/* Code for Assignment ?? 
 * Name:
 * Usercode:
 * ID:
 */

import ecs100.*;
import java.util.*;
import java.io.*;
import java.awt.*;
import java.awt.image.*;
import javax.imageio.ImageIO;

/** <description of class Main>
 */
public class Main{

    private Arm arm;
    private Drawing drawing;
    private ToolPath tool_path;
    // state of the GUI
    private int state; // 0 - nothing
    // 1 - inverse point kinematics - point
    // 2 - enter path. Each click adds point  
    // 3 - enter path pause. Click does not add the point to the path
    private int lastState;
    /**      */
    public Main(){
        UI.initialise();
        UI.addButton("xy to angles", this::inverse);
        UI.addButton("Enter path XY", this::enter_path_xy);
        UI.addButton("Save path XY", this::save_xy);
        UI.addButton("Load path XY", this::load_xy);
        UI.addButton("Save path Ang", this::save_ang);
        UI.addButton("Load path Ang:Play", this::load_ang);
        UI.addButton("save the pwm", this::savepwmfile);
        UI.addButton("smoth drawing",()->{state=4;});
        UI.addButton("square", this::doSquare);
        UI.addButton("circle", this::doCircle);
        UI.addButton("image", this::loadImage);
        // UI.addButton("Quit", UI::quit);
        UI.setMouseMotionListener(this::doMouse);
        UI.setKeyListener(this::doKeys);

        //ServerSocket serverSocket = new ServerSocket(22); 
        this.arm = new Arm();
        this.drawing = new Drawing();
        this.run();
        arm.draw();
    }

    public void savepwmfile(){
        tool_path=new ToolPath();
        tool_path.convert_drawing_to_angles(drawing,arm,"");
        tool_path.convert_angles_to_pwm(arm);
        tool_path.save_pwm_file();
    }

    public void doKeys(String action){
        UI.printf("Key :%s \n", action);
        if (action.equals("b")) {
            // break - stop entering the lines
            lastState=state;

            state = 3;

            //

        }

    }

    public void doSquare() {
        doMouse("clicked", 267, 147);
        doMouse("clicked", 327, 147);
        doMouse("clicked", 327, 227);
        doMouse("clicked", 267, 207);
        doMouse("clicked", 267, 147);
    }

    public void doCircle(){
        double centreX = 340;
        double centreY = 140;
        double a = 38.0;
        double b = 35.0;
        double r = 37.5;
        for (double x = centreX-a; x < centreX+a; x+=10) {
            double y = Math.sqrt(b*b*(1-(Math.pow(x-centreX,2)/a/a)))+centreY;
            doMouse("clicked", x, y);
        }
        for (double x = centreX+a; x >= centreX-a; x-=10) {
            double y = -Math.sqrt(b*b*(1-(Math.pow(x-centreX,2)/a/a)))+centreY;
            doMouse("clicked", x, y);
        }
        for (double x = centreX-a; x <= centreX+a; x+=10) {
            double y = Math.sqrt(b*b*(1-(Math.pow(x-centreX,2)/a/a)))+centreY;
            doMouse("clicked", x, y);
        }

    }

    public void loadImage() {

        try{
            BufferedImage image = ImageIO.read(new File("elf.jpg"));
            int [][] pixels = new int[image.getWidth()][];

            for (int x = 0; x < image.getWidth(); x++) {
                pixels[x] = new int[image.getHeight()];

                for (int y = 0; y < image.getHeight(); y++) {
                    pixels[x][y] = (int) (image.getRGB(x, y) == 0xFFFFFFFF ? 0 : 1);
                }
            }

            double left = 240;
            double top = 100;
            double scale = 1;
            int lastValue = 0;
            int count = 0;

            for (int row = 0; row < pixels.length; row++) {
                if (row %2 == 0){
                    for (int col = 0; col < pixels[0].length; col++) {

                        int currentValue = pixels[row][col];
                        if (lastValue != currentValue ) {
                            if (currentValue == 1) {
                                lastState=state;
                                state = 3;
                                state = lastState;
                                arm.inverseKinematic(left+col,top+row);
                                if (arm.valid_state) {
                                    drawing.add_point_to_path(left+col,top+row,false); // add point wit pen up     
                                }
                            }
                            else {
                                arm.inverseKinematic(left+col,top+row);
                                if (arm.valid_state) {
                                    drawing.add_point_to_path(left+col,top+row,true); // add point with pen down
                                }
                            }
                            lastValue = currentValue;
                            count++;
                        }

                    }
                }
                else {
                    for (int col = pixels[0].length-1; col >= 0; col--) {

                        int currentValue = pixels[row][col];
                        if (lastValue != currentValue ) {
                            if (currentValue == 1) {
                                lastState=state;
                                state = 3;
                                state = lastState;
                                arm.inverseKinematic(left+col,top+row);
                                if (arm.valid_state) {
                                    drawing.add_point_to_path(left+col,top+row,false); // add point wit pen up     
                                }
                            }
                            else {
                                arm.inverseKinematic(left+col,top+row);
                                if (arm.valid_state) {
                                    drawing.add_point_to_path(left+col,top+row,true); // add point with pen down
                                }
                            }
                            lastValue = currentValue;
                            count++;
                        }

                    }
                }
            }
            UI.println(count);
            UI.printMessage("DONE");
        }
        catch (IOException e) {UI.println("File loading failed: "+e);}       

    }

    public void doMouse(String action, double x, double y) {
        //UI.printf("Mouse Click:%s, state:%d  x:%3.1f  y:%3.1f\n",
        //   action,state,x,y);
        UI.clearGraphics();
        String out_str=String.format("%3.1f %3.1f",x,y);
        UI.drawString(out_str, x+10,y+10);
        // 
        if ((state == 1)&&(action.equals("clicked"))){
            // draw as 

            arm.inverseKinematic(x,y);
            arm.draw();
            return;
        }

        if ( ((state == 2)||(state == 3)||(state==4))&&action.equals("moved") ){
            // draw arm and path
            arm.inverseKinematic(x,y);
            arm.draw();

            // draw segment from last entered point to current mouse position
            if (((state == 2)||(state==4))&&(drawing.get_path_size()>0)){
                PointXY lp = new PointXY();
                lp = drawing.get_path_last_point();
                //if (lp.get_pen()){
                UI.setColor(Color.GRAY);
                UI.drawLine(lp.get_x(),lp.get_y(),x,y);
                // }
            }
            drawing.draw();
        }

        // add point
        if (   (state == 2) &&(action.equals("clicked"))){
            // add point(pen down) and draw
            UI.printf("Adding point x=%f y=%f\n",x,y);
            arm.inverseKinematic(x,y);
            if (arm.valid_state) {
                drawing.add_point_to_path(x,y,true); // add point with pen down
                arm.draw();
                drawing.draw();
                drawing.print_path();
            }
        }

        if (   (state == 3) &&(action.equals("pressed"))){
            // add point and draw
            //UI.printf("Adding point x=%f y=%f\n",x,y);
            state = lastState;
            arm.inverseKinematic(x,y);
            if (arm.valid_state) {
                drawing.add_point_to_path(x,y,false); // add point wit pen up     
                arm.draw();
                drawing.draw();
                drawing.print_path();

            }
        }
        if(  (state == 4) &&(action.equals("pressed"))){
            arm.inverseKinematic(x,y);
            if (arm.valid_state) {
                drawing.add_point_to_path(x,y,true); // add point with pen down
                arm.draw();
                drawing.draw();
                drawing.print_path();
            }
        }
        if (   (state == 4) &&(action.equals("dragged"))){
            arm.inverseKinematic(x,y);
            if (arm.valid_state) {
                drawing.add_point_to_path(x,y,true); // add point with pen down
                arm.draw();
                drawing.draw();
                drawing.print_path();
            }
        }
    }

    public void save_xy(){
        state = 0;
        String fname = UIFileChooser.save();
        if(fname!=null){
            drawing.save_path(fname);
        }

    }

    public void enter_path_xy(){
        state = 2;
    }

    public void inverse(){
        state = 1;
        arm.draw();
    }

    public void load_xy(){
        state = 0;
        String fname = UIFileChooser.open();
        drawing.load_path(fname);
        drawing.draw();

        arm.draw();
    }

    // save angles into the file
    public void save_ang(){
        String filename=UIFileChooser.save();

        // String filename=UIFileChooser.save();
        if(filename!=null){
            try{
                tool_path.convert_drawing_to_angles(drawing,arm,filename);
                PrintStream ps=new PrintStream(new File(filename));
                ArrayList<Double> theta1_vector =tool_path.getTheta1_vector();
                ArrayList<Double> theta2_vector =tool_path.getTheta2_vector();
                for(int i=0;i<theta1_vector.size();i++){
                    double theta1=theta1_vector.get(i);
                    double theta2=theta2_vector.get(i);                  
                    String thta1=theta1 + "";
                    String thta2=theta2 + "";
                    ps.println(thta1+","+thta2);
                }
                ps.close();
            }
            catch(IOException e){
                UI.printf("File Failure % \n", e);
            }
        }
    }

    public void load_ang(){
    }

    public void run() {
        while(true) {
            arm.draw();
            UI.sleep(20);
        }
    }

    public static void main(String[] args){
        Main obj = new Main();
    }    

}
