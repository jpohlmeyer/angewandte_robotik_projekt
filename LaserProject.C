/*
 * ===========================================================================
 *
 * example_template.C --
 *
 *
 * Jan Moringen <jmoringe@techfak.uni-bielefeld.de>
 *
 *    Copyright (C) 2007
 *    Computer Engineering Group
 *    Faculty of Technology
 *    Bielefeld University
 *
 * 1.0 / 15. Mar 07 (jm)
 * - from scratch
 *
 *
 * ===========================================================================
 */

#include <AdvancedTools.H>

#include <DisplayTools.H>

#include <VideoDisplay.H>

#include <ScannerMapFull.H>

using namespace std;

using namespace tools;

using namespace scannermap;

Log log_("main");

bool terminate_ = false;

bool autoStop = false;

//
// This signal handler function is called when the process receives the SIGINT
// signal (this happens, for example, when ctrl+c is pressed). When the
// handler is called for the first time, it tries to terminate the main loop
// gracefully, when it is called twice, it terminates the process forcefully.
//
void signalHandler(int signal) {
    static bool secondTime = false;

    const char *msg1 = "caught signal\n";
    int res = write(STDERR_FILENO, msg1, strlen(msg1));
    terminate_ = true;

    if (secondTime) {
        const char *msg2 = "caught signal again; dying right now\n";
        res = write(STDERR_FILENO, msg2, strlen(msg2));
        _exit(-1);
    }

    secondTime = true;
    // avoid the "unused variable" warning
    assert(res == res);
}


//
// Initialization of the Matrix library
//
template<>
ns_matrix::MultOpt<double> ns_matrix::Matrix<double>::multOpt = ns_matrix::MultOpt<double>();


//
// Histogram class
//

class Histogram : public display::Displayable
{
    public:
        Histogram(double pointR) {}
        virtual ~Histogram() {}

        //virtual void setBins(int bins[]);

    private:
        virtual void draw(display::Graphics &graphics);

        //int bins[];
        
};

/*void Histogram::setBins(int bins[]) {
    this->bins = bins;
}*/

void Histogram::draw(display::Graphics &graphics) {
    display::Color drawCol( 1.0, 0.5, 0.0 );

    graphics.setColor( drawCol );
    /*for (int i=0; i<bins.size(); ++i) {
        graphics.drawBox( Rectangle( Vector<double>(i*10,bins[]), Vector<double>(i*10+10,0)));
    }*/
}

//
// Main function
//
int main(int argc, char* argv[]) {
    //
    // Initialization stuff
    //
    scannermap::init();

    display::init();

    videodisplay::init();

    //
    // We accept one or two arguments: The first is the Scanner and Steering
    // implementation, the second, optional, argument indicates, whether the
    // RemoteScanner object has to initialise the scanner hardware.
    //
    if (argc < 2) {
        log_ << Log::ERROR << "too few arguments" << endl;
        cerr << "usage: LaserProject IMPLEMENTATION [INIT]" << endl
            << "where IMPLEMENTATION is one of" << endl
            << "remote" << endl
            << "vr" << endl;
        return EXIT_FAILURE;
    }

    //
    // We allocate a Scanner object of the specified type. Using the pointers to
    // the derived classes, we can call the implementation's special member
    // functions to setup the specific properties of the selected
    // implementation.
    //
    scannermap::Scanner* scanner = 0;

    if (string(argv[1]) == "remote") {
        RemoteScanner* temp = new RemoteScanner();

        if (argc == 3 && string(argv[2]) == "init")
            temp->setDoInitScanner(true);

        scanner = temp;
    } else if (string(argv[1]) == "vr") {
        VRScanner* temp = new VRScanner();
        temp->setSize(361);

        temp->setNoiseEnabled(false);
        scanner = temp;
    } else {
        log_ << Log::ERROR << "unrecognized scanner implementation" << endl;
        return EXIT_FAILURE;
    }

    //
    // We allocate a Steering object of the same type as the Scanner
    // object. Again, the specific properties are adjusted before the type is
    // generalized to the Steering interface..
    //
    scannermap::Steering* steering = 0;

    if (string(argv[1]) == "remote") {
        steering = new RemoteSteering();
    } else if (string(argv[1]) == "vr") {
        VRSteering* temp = new VRSteering();

        temp->setVRWorldFile("labor.vr");
        temp->setPose(Pose(1.0, -3.0, 0.0));

        steering = temp;
    } else {
        if (scanner) delete scanner;

        log_ << Log::ERROR << "unrecognized steering implementation" << endl;
        return EXIT_FAILURE;
    }

    //
    // The following objects will hold the gathered and computed data:
    // - scan:      The last scan which was retrieved from the Scanner object
    // - map:       The environment map.
    // - indicator: The robot position.
    // - histogram: An example histogram.
    //
    ScanData scan;
    Map map;
    Indicator indicator;
    Histogram histogram(5);

    //
    // We use some displays to show these data objects. The \a vrWindow display
    // is only useful when working with the \a vr implementations.
    //
    display::Display* scanWindow = 0;
    display::Display* mapWindow = 0;
    display::Display* vrWindow = 0;
    display::Display* histWindow = 0;

    try {
        //
        // We add the scan data object to the first display and make sure the
        // correct aspect ratio is maintained.
        //
        scanWindow = display::Display::getFactory().createInst("video_window",
                string("Scan Window"), 500, 500);

        display::Display::DisplayItem& scanItem = scanWindow->add(scan, "scan");
        scanItem.setFlags(display::Display::DisplayItem::FLAG_NOSTRETCH);

        //
        // The second display shows the map and indicator data objects. The
        // indicator object is synchronized to the map object, thus it is display
        // at the some position and at the same scale as the map object. For both
        // objects the aspect ratio is maintained.
        //
        mapWindow = display::Display::getFactory().createInst("video_window",
                string("Map Window"), 500, 500);

        display::Display::DisplayItem& mapItem = mapWindow->add(map, "map");
        mapItem.setFlags(display::Display::DisplayItem::FLAG_NOSTRETCH);

        display::Display::DisplayItem& robotItem = mapWindow->add(indicator, "robot");
        robotItem.synchronizeTo(&mapItem);

        //
        // Histogram window
        //
        histWindow = display::Display::getFactory().createInst("video_window",
                string("Histogram Window"), 500, 300);

        display::Display::DisplayItem& histItem = histWindow->add(histogram, "histogram");
        histItem.setFlags(display::Display::DisplayItem::FLAG_NOSTRETCH);
        histItem.setCoordBounds( Rectangle( 0, 0, 500, 300 ) );

        //
        // If the \a vr implementations are used, we add a third display showing
        // the simulated block world the robot position inside this virtual
        // environment. As before, the indicator object has to be synchronized to
        // the map object.
        //
        if (string(argv[1]) == "vr") {
            vrWindow = display::Display::getFactory().createInst("video_window",
                    string("VR Window"), 500, 500);

            display::Display::DisplayItem& worldItem
                = vrWindow->add(*Blackboard::getInstance().get<VRWorld>(Path("/scannermap/vr/world")),
                        "world");
            worldItem.setFlags(display::Display::DisplayItem::FLAG_NOSTRETCH);

            display::Display::DisplayItem& vrRobotItem
                = vrWindow->add(*(new Indicator(*Blackboard::getInstance().get<Pose>(Path("/scannermap/vr/pose")))),
                        "robot");
            vrRobotItem.synchronizeTo(&worldItem);
        }
    } catch (const Exception& exception) {
        log_ << Log::ERROR << "could not setup display stuff; reason: "
            << exception.getMessage() << endl;

        if (scanner)    delete scanner;
        if (steering)   delete steering;
        if (scanWindow) delete scanWindow;
        if (mapWindow)  delete mapWindow;
        if (vrWindow)   delete vrWindow;
        if (histWindow) delete histWindow;

        return EXIT_FAILURE;
    }

    //
    // This installs the signal handler before the main loop starts.
    //
    signal(SIGINT, signalHandler);

    //
    // If anything goes wrong in the main loop below, the exception handler
    // should cleanup the mess.
    //
    try {
        //
        // Please do not disable this function unless you can be certain that
        // no collisions can happen.
        //
        steering->setAutoStop(autoStop);

        //
        // Some static movement commands. The function calls block until the robot
        // finishes its movement. So use these functions with great care, or even
        // better, do not use them at all. Use setWheelSpeed instead.
        //
        //steering->turn(PI / 4.0);

        //steering->move(1.0);

        //
        // A little main loop, that
        // - sets a movement direction
        // - retrieves scan data from the Scanner object
        // - updates the displays
        //
        while (!terminate_) {
            //
            //
            //
            //if (!steering->didAutoStop())
            //steering->setWheelSpeed(0.15, 0.2);

            //
            //
            //
            scanner->scan(scan);

            unsigned int minIdx = scan.getMin().first;
            float minDist = 0.7;
            int left = 0, right = 0, front = 0;
            int scanSize3 = scan.size()/3;
            for (int i=0; i < scanSize3; ++i) {
                if (!scan[i].isValid()) // Skip invalid points
                    continue;
                if (scan[i].getDistance() <= minDist) {
                    right++;
                }
                if (scan[i + scanSize3].getDistance() <= minDist) {
                    front++;
                }
                if (scan[i + 2 * scanSize3].getDistance() <= minDist) {
                    left++;
                }
            }

            if (front > 5) {
                if (abs(right - left) < 5) {
                    if (right < 5) {
                        steering->setWheelSpeed(0.2, 0.0);
                    } else {
                        steering->turn(PI);
                    }
                } else if (right > left) {
                    steering->setWheelSpeed(0.0, 0.2);
                } else if (left > right) {
                    steering->setWheelSpeed(0.2, 0.0);
                } 
            } else if (scan[minIdx].getDistance() <= minDist) { 
                if (minIdx < scan.size()/2) {
                    steering->setWheelSpeed(0.1, 0.2);
                } else {
                    steering->setWheelSpeed(0.2, 0.1);
                }
            } 
            else {
                steering->setWheelSpeed(0.15, 0.15);
            }

            double angle;
            int bin = 50;
            int hist[bin];
            for (int i=0; i < bin; ++i) {
                hist[i] = 0;
            }
            for (unsigned int i=0; i < scan.size()-1; ++i) {
                if (scan[i].isValid() && scan[i+1].isValid()) {
                    angle = atan2(scan[i][1] - scan[i+1][1], scan[i][0] - scan[i+1][0]) * 180 /PI;
                    int j = ((int) ((angle+180)/(360.0/bin)));
                    hist[j] = hist[j] + 1;
                }
            }

            for (int i = 0; i < bin; ++i) {
                std::cout<<"bin "<<i<<" : "<<hist[i]<<endl;
            }
            //histogram->setBins(hist);

            scanWindow->update();
            mapWindow->update();
            if (vrWindow)
                vrWindow->update();
            histWindow->update();

            usleep(100000);
        }
    } catch (const Exception& exception) {
        log_ << Log::ERROR << "something went wrong: "
            << exception.getMessage() << endl;

        if (scanner)    delete scanner;
        if (steering)   delete steering;
        if (scanWindow) delete scanWindow;
        if (mapWindow)  delete mapWindow;
        if (vrWindow)   delete vrWindow;
        if (histWindow) delete histWindow;

        return EXIT_FAILURE;
    }

    //
    // Cleanup
    //
    if (scanner)    delete scanner;
    if (steering)   delete steering;
    if (scanWindow) delete scanWindow;
    if (mapWindow)  delete mapWindow;
    if (vrWindow)   delete vrWindow;
    if (histWindow) delete histWindow;

    //
    //
    //
    return EXIT_SUCCESS;
}
