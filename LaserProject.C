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

#include <KeyInput.H>

#include <math.h>

#include <limits>

#define BINCOUNT 300

#define BINCOUNTDIST 500

#define SEARCHDEGREE 15.0

#define MAXLASERDIST 9.0

#define SEARCHDIST 1.0

#define ANGLENOISECONST 10

#define NOISECONST 4

using namespace std;

using namespace tools;

using namespace scannermap;

Log log_("main");

bool terminate_ = false;

bool autoStop = false;


void averageScanData(ScanData &scan, double *elementsX, double *elementsY) {
    for (int j = 0; j < NOISECONST; ++j) {
        elementsX[j] = scan[j][0];
        elementsX[scan.size()-1-j] = scan[scan.size()-1-j][0];
        elementsY[j] = scan[j][1];
        elementsY[scan.size()-1-j] = scan[scan.size()-1-j][1];
    }
    for (unsigned int i = NOISECONST; i < scan.size()-NOISECONST; ++i) {
        int numNotValid = 0;
        double x = 0, y = 0;
        if (scan[i].isValid()) {
            x = scan[i][0];
            y = scan[i][1];
        } else {
            numNotValid++;
        }
        for (int j = 1; j <= NOISECONST; ++j) {
            if (scan[i-j].isValid()) {
                x = x + scan[i-j][0];
                y = y + scan[i-j][1];
            } else {
                numNotValid++;
            }
            if (scan[i+j].isValid()) {
                x = x + scan[i+j][0];
                y = y + scan[i+j][1];
            } else {
                numNotValid++;
            }
        }
        if (numNotValid == 2*NOISECONST+1) {
            x = numeric_limits<double>::quiet_NaN();
            y = numeric_limits<double>::quiet_NaN();
        } else {
            x = x/((2*NOISECONST+1)-numNotValid);
            y = y/((2*NOISECONST+1)-numNotValid);
        }
        elementsX[i] = x;
        elementsY[i] = y;
        //cout<<i<<": scan x: "<<scan[i][0]<<" scan y: "<<scan[i][1]<<" elementsX: "<<elementsX[i]<<" elementsY: "<<elementsY[i]<<endl;
    }
}
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
        Histogram(display::Color drawCol, int bincount, double* bins) { this->drawCol = drawCol; this->bincount = bincount; this->bins = bins; }
        virtual ~Histogram() {}

        //virtual void setBins(int bins[]);
        double* bins;

    private:
        virtual void draw(display::Graphics &graphics);

        display::Color drawCol;
        int bincount;
        //int bins[];

};

/*void Histogram::setBins(int bins[]) {
  this->bins = bins;
  }*/

void Histogram::draw(display::Graphics &graphics) {

    graphics.setColor( drawCol );
    for (int i=0; i<bincount; ++i) {
        graphics.fillCircle( Vector<double>((i*(490.0/bincount))+(250.0/bincount),bins[i]), 5*250.0/bincount, display::Graphics::SCALE_ABSOLUTE );

    }
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

        //temp->setNoiseConst(0.03);
        temp->setNoiseLinear(0.005);
        temp->setNoiseEnabled(true);
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
    ScanData oldScan;
    Map map;
    Indicator indicator;
    //display::Color drawCol( 1.0, 0.5, 0.0 );
    //Histogram histogram(drawCol);
    //display::Color drawColOld( 0.5, 1.0, 0.0 );
    //Histogram histogramOld(drawColOld);
    //display::Color drawColCor( 0.0, 0.5, 1.0 );
    //Histogram histogramCor(drawColCor);

    display::Color drawColOld( 0.0, 0.0, 1.0 );
    double binsOld[BINCOUNT];
    Histogram histogramOld(drawColOld,BINCOUNT,&binsOld[0]);

    display::Color drawCol( 0.0, 1.0, 0.0 );
    double bins[BINCOUNT];
    Histogram histogram(drawCol,BINCOUNT,&bins[0]);

    display::Color drawColCor( 1.0, 0.0, 0.0 );
    double binsCor[BINCOUNT];
    Histogram histogramCor(drawColCor,BINCOUNT,&binsCor[0]);


    display::Color drawColOldY( 0.0, 0.0, 1.0 );
    double binsOldY[BINCOUNTDIST];
    Histogram histogramOldY(drawColOldY,BINCOUNTDIST,&binsOldY[0]);

    display::Color drawColY( 0.0, 1.0, 0.0 );
    double binsY[BINCOUNTDIST];
    Histogram histogramY(drawColY,BINCOUNTDIST,&binsY[0]);

    display::Color drawColCorY( 1.0, 0.0, 0.0 );
    double binsCorY[BINCOUNTDIST];
    Histogram histogramCorY(drawColCorY,BINCOUNTDIST,&binsCorY[0]);


    display::Color drawColOldX( 0.0, 0.0, 1.0 );
    double binsOldX[BINCOUNTDIST];
    Histogram histogramOldX(drawColOldX,BINCOUNTDIST,&binsOldX[0]);

    display::Color drawColX( 0.0, 1.0, 0.0 );
    double binsX[BINCOUNTDIST];
    Histogram histogramX(drawColX,BINCOUNTDIST,&binsX[0]);

    display::Color drawColCorX( 1.0, 0.0, 0.0 );
    double binsCorX[BINCOUNTDIST];
    Histogram histogramCorX(drawColCorX,BINCOUNTDIST,&binsCorX[0]);

    //
    // We use some displays to show these data objects. The \a vrWindow display
    // is only useful when working with the \a vr implementations.
    //
    display::Display* scanWindow = 0;
    display::Display* mapWindow = 0;
    display::Display* vrWindow = 0;
    display::Display* histWindowX = 0;
    display::Display* histWindowY = 0;
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

        //display::Display::DisplayItem& robotItem = mapWindow->add(indicator, "robot");
        //robotItem.synchronizeTo(&mapItem);

        //
        // Histogram window
        //
        histWindow = display::Display::getFactory().createInst("video_window",
                string("Histogram Window"), 500, 300);
        histWindowX = display::Display::getFactory().createInst("video_window",
                string("HistogramX Window"), 500, 300);
        histWindowY = display::Display::getFactory().createInst("video_window",
                string("HistogramY Window"), 500, 300);

        display::Display::DisplayItem& histItemCorY = histWindowY->add(histogramCorY, "histogramCorY");
        histItemCorY.setFlags(display::Display::DisplayItem::FLAG_NOSTRETCH);
        histItemCorY.setCoordBounds( Rectangle( 0, 0, 500, 300 ) );
        display::Display::DisplayItem& histItemY = histWindowY->add(histogramY, "histogramY");
        histItemY.setFlags(display::Display::DisplayItem::FLAG_NOSTRETCH);
        histItemY.setCoordBounds( Rectangle( 0, 0, 500, 300 ) );
        display::Display::DisplayItem& histItemOldY = histWindowY->add(histogramOldY, "histogramOldY");
        histItemOldY.setFlags(display::Display::DisplayItem::FLAG_NOSTRETCH);
        histItemOldY.setCoordBounds( Rectangle( 0, 0, 500, 300 ) );

        display::Display::DisplayItem& histItemX = histWindowX->add(histogramX, "histogramX");
        histItemX.setFlags(display::Display::DisplayItem::FLAG_NOSTRETCH);
        histItemX.setCoordBounds( Rectangle( 0, 0, 500, 300 ) );
        display::Display::DisplayItem& histItemOldX = histWindowX->add(histogramOldX, "histogramOldX");
        histItemOldX.setFlags(display::Display::DisplayItem::FLAG_NOSTRETCH);
        histItemOldX.setCoordBounds( Rectangle( 0, 0, 500, 300 ) );
        display::Display::DisplayItem& histItemCorX = histWindowX->add(histogramCorX, "histogramCorX");
        histItemCorX.setFlags(display::Display::DisplayItem::FLAG_NOSTRETCH);
        histItemCorX.setCoordBounds( Rectangle( 0, 0, 500, 300 ) );

        display::Display::DisplayItem& histItem = histWindow->add(histogram, "histogram");
        display::Display::DisplayItem& histItemOld = histWindow->add(histogramOld, "histogramOld");
        display::Display::DisplayItem& histItemCor = histWindow->add(histogramCor, "histogramCor");
        histItem.setFlags(display::Display::DisplayItem::FLAG_NOSTRETCH);
        histItemOld.setFlags(display::Display::DisplayItem::FLAG_NOSTRETCH);
        histItemCor.setFlags(display::Display::DisplayItem::FLAG_NOSTRETCH);
        histItem.setCoordBounds( Rectangle( 0, 0, 500, 300 ) );
        histItemOld.setCoordBounds( Rectangle( 0, 0, 500, 300 ) );
        histItemCor.setCoordBounds( Rectangle( 0, 0, 500, 300 ) );

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
        if (histWindowY) delete histWindowY;
        if (histWindowX) delete histWindowX;

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
        scanner->scan(scan);
        //map.integrate(scan);
        ScanData obstacles = scan;

        //old angle, x and y histograms
        double oldHist[BINCOUNT];
        int oldHistX[BINCOUNTDIST];
        int oldHistY[BINCOUNTDIST];

        //initialise histograms with zero
        for (int i=0; i < BINCOUNT; ++i) {
            oldHist[i] = 0;
        }
        for (int i=0; i < BINCOUNTDIST; ++i) {
            oldHistX[i] = 0;
            oldHistY[i] = 0;
        }

        double elementsX[scan.size()];
        double elementsY[scan.size()];

        averageScanData(scan, &elementsX[0], &elementsY[0]); 
        //calculate angle histogram
        double angle;
        for (unsigned int i=ANGLENOISECONST; i < scan.size()-ANGLENOISECONST; ++i) {
            if (!isnan(elementsX[i-ANGLENOISECONST]) && !isnan(elementsX[i+ANGLENOISECONST])) {
                angle = atan2(elementsY[i-ANGLENOISECONST] - elementsY[i+ANGLENOISECONST], elementsX[i-ANGLENOISECONST] - elementsX[i+ANGLENOISECONST]) * 180 /PI;
                int j = ((int) ((angle+180)/(360.0/BINCOUNT)));
                oldHist[j] = oldHist[j] + 1;
            }
        }

        /*
        //calculate angle histogram
        double angle;
        for (unsigned int i=ANGLENOISECONST; i < scan.size()-ANGLENOISECONST; ++i) {
        if (scan[i-ANGLENOISECONST].isValid() && scan[i+ANGLENOISECONST].isValid()) {
        angle = atan2(scan[i-ANGLENOISECONST][1] - scan[i+ANGLENOISECONST][1], scan[i-ANGLENOISECONST][0] - scan[i+ANGLENOISECONST][0]) * 180 /PI;
        int j = ((int) ((angle+180)/(360.0/BINCOUNT)));
        oldHist[j] = oldHist[j] + 1;
        }
        }*/

        //calculate most common direction in angle histogram
        int maxI = 0;
        int maxHist = 0;
        for (int i = 0; i < BINCOUNT; ++i) {
            if (oldHist[i] > maxHist) {
                histogramOld.bins[i] = oldHist[i];
                maxHist = oldHist[i];
                maxI = i;
            }
        }

        //calculat initial rotation offsett needet to aligne the map (most common direction) to the x ynd y axis
        double transOffsetX = 0;
        double transOffsetY = 0;
        double rotationOffset;
        if (maxI > BINCOUNT/2) {
            rotationOffset = 2.0*PI-((((maxI-BINCOUNT/2) * 360.0) / BINCOUNT)*PI/180.0);
        } else {
            rotationOffset = (((BINCOUNT/2-maxI) * 360.0) / BINCOUNT)*PI/180.0;
        }

        std::cout<<"initial offset: "<<rotationOffset<<" maxJ "<<maxI<<endl;

        //rotate initial scan so it is axis aligned
        scan.rotate(rotationOffset);

        //take the mean of the scan data
        averageScanData(scan, &elementsX[0], &elementsY[0]); 

        //calculate x and y histograms
        for (unsigned int i = 0; i < scan.size(); ++i) {
            if (!isnan(elementsX[i])) {
                int j = (int) ((elementsX[i] * BINCOUNTDIST/2.0) / MAXLASERDIST) + (BINCOUNTDIST/2.0);
                oldHistX[j] = oldHistX[j]+1;
                j = (int) ((elementsY[i] * BINCOUNTDIST/2.0) / MAXLASERDIST) + (BINCOUNTDIST/2.0);
                oldHistY[j] = oldHistY[j]+1;
            }
        }

        /*calculate x and y histogram
          for (unsigned int i = 0; i < scan.size(); ++i) {
          if (scan[i].isValid()) {
          int j = (int) ((scan[i][0] * BINCOUNTDIST/2) / MAXLASERDIST) + (BINCOUNTDIST/2);
          oldHistX[j] = oldHistX[j]+1;
          j = (int) ((scan[i][1] * BINCOUNTDIST/2) / MAXLASERDIST) + (BINCOUNTDIST/2);
          oldHistY[j] = oldHistY[j]+1;
          }
          }*/

        /*
           for (int i = 0; i < BINCOUNTDIST; ++i) {
           cout<<"initial x "<<i<<" : "<<oldHistX[i]<<endl;
           } 
           for (int i = 0; i < BINCOUNTDIST; ++i) {
           cout<<"y "<<i<<" : "<<oldHistY[i]<<endl;
           }*/

        //integrate initial axis aligned scan, show  map and initial histogram and scan
        map.integrate(scan);
        mapWindow->update();
        histWindow->update();
        histWindowX->update();
        histWindowY->update();
        scanWindow->update();

        //get current odom Position
        Pose odom = steering->getPosition();
        Pose oldPos;

        //Counter to make map update only every couple of rounds
        int count = 0;
        //main execution loop-----------------------------------------------------------------------------------
        while (!terminate_) {
            //
            //if (!steering->didAutoStop())
            //steering->setWheelSpeed(0.15, 0.2);
            //

            //get current scan for obstacle avoidance
            scanner->scan(obstacles);

            //movement with obstacle avoidance
            unsigned int minIdx = obstacles.getMin().first;
            float minDist = 0.7;
            int left = 0, right = 0, front = 0;
            int scanSize3 = obstacles.size()/3;
            for (int i=0; i < scanSize3; ++i) {
                if (!obstacles[i].isValid()) // Skip invalid points
                    continue;
                if (obstacles[i].getDistance() <= minDist) {
                    right++;
                }
                if (obstacles[i + scanSize3].getDistance() <= minDist) {
                    front++;
                }
                if (obstacles[i + 2 * scanSize3].getDistance() <= minDist) {
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
            } else if (obstacles[minIdx].getDistance() <= minDist) { 
                if (minIdx < scan.size()/2) {
                    steering->setWheelSpeed(0.1, 0.2);
                } else {
                    steering->setWheelSpeed(0.2, 0.1);
                }
            } 
            else {
                steering->setWheelSpeed(0.15, 0.15);
            }

            //current test movement
            //steering->setWheelSpeed(0.15, 0.05);
            //steering->move(0.5);

            //sup execution loop responsible for map update
            if (count % 5 == 0) {
                //save odom position from previous update step and get new odom pos
                oldPos = odom;
                odom = steering->getPosition();

                //save scan from previous update step and get current scan
                oldScan = scan;
                //scan = obstacles;
                scanner->scan(scan); 

                //set up and initialise current histograms with zero
                double hist[BINCOUNT];
                int histX[BINCOUNTDIST];
                int histY[BINCOUNTDIST];
                for (int i=0; i < BINCOUNT; ++i) {
                    hist[i] = 0;
                }
                for (int i=0; i < BINCOUNTDIST; ++i) {
                    histX[i] = 0;
                    histY[i] = 0;
                }

                averageScanData(scan, &elementsX[0], &elementsY[0]);

                //calculate current angle histogram
                for (unsigned int i=ANGLENOISECONST; i < scan.size()-ANGLENOISECONST; ++i) {
                    if (!isnan(elementsX[i-ANGLENOISECONST]) && !isnan(elementsX[i+ANGLENOISECONST])) {
                        angle = atan2(elementsY[i-ANGLENOISECONST] - elementsY[i+ANGLENOISECONST], elementsX[i-ANGLENOISECONST] - elementsX[i+ANGLENOISECONST]) * 180 /PI;
                        int j = ((int) ((angle+180)/(360.0/BINCOUNT)));
                        hist[j] = hist[j] + 1;
                    }
                }

                /*
                //calculate current angle histogram
                for (unsigned int i=ANGLENOISECONST; i < scan.size()-ANGLENOISECONST; ++i) {
                if (scan[i-ANGLENOISECONST].isValid() && scan[i+ANGLENOISECONST].isValid()) {
                angle = atan2(scan[i-ANGLENOISECONST][1] - scan[i+ANGLENOISECONST][1], scan[i-ANGLENOISECONST][0] - scan[i+ANGLENOISECONST][0]) * 180 /PI;
                int j = ((int) ((angle+180)/(360.0/BINCOUNT)));
                hist[j] = hist[j] + 1;
                }
                }*/

                //calculate correlation between old and current angle histogram
                //save max correlation
                double corr[BINCOUNT];
                double maxCor = 0;
                for (int j = 0; j < BINCOUNT; ++j) {
                    corr[j] = 0;
                    for (int i = 0; i < BINCOUNT; ++i) {
                        corr[j] = corr[j] + oldHist[i] * hist[(i+j) % BINCOUNT];
                    }
                    if (corr[j] > maxCor) {
                        maxCor = corr[j];
                    }
                }

                //set up info to draw histograms
                //current angle hist becomes old angle hist
                int maxIdx = 0;
                int maxVal = 0;
                for (int i = 0; i < BINCOUNT; ++i) {
                    histogram.bins[i] = hist[i];
                    histogramOld.bins[i] = oldHist[i];
                    histogramCor.bins[i] = (((double) corr[i])/maxCor)*290;
                    oldHist[i] = hist[i];
                    if (hist[i] > maxVal) {
                        maxVal = hist[i];
                        maxIdx = i;
                    }
                    std::cout<<"maxIdx: "<<maxIdx<<endl;
                }

                // ROTATION CORRECTION----------------------------------------------------------------------------------------
                //get estimated orientation from odometrie as center for max correlation search
                double turnRad = odom.getOrientation() - oldPos.getOrientation();
                if (turnRad < 0) {
                    turnRad = 2.0 * PI + turnRad;
                }
                //translat to degrees
                double turn = turnRad * 180 / PI;
                //calculate bin that relates to estimated turn to search in 
                int searchPointIdx = (int) ((360.0-turn)/(360.0/BINCOUNT));

                //calculate how many bins to search around the estimated search bin
                int binsFromDegree = (int) (SEARCHDEGREE/(360.0/BINCOUNT));
                //search for max correlation around searchPointIdx
                //save relative distance from bin with max from origin of search
                int corrMax = 0;
                double corrMaxVal = 0;
                for(int i = 0; i < binsFromDegree; ++i) {
                    if (corr[(searchPointIdx - i + BINCOUNT)%BINCOUNT] > corrMaxVal) {
                        corrMax = -i;
                        corrMaxVal = corr[(searchPointIdx - i + BINCOUNT)%BINCOUNT];
                    } 
                    if (corr[(searchPointIdx+i)%BINCOUNT] > corrMaxVal) {
                        corrMax = i;
                        corrMaxVal = corr[(searchPointIdx + i)%BINCOUNT];
                    }
                }

                //update searchPointIdx to idx with max corr
                searchPointIdx = (searchPointIdx + corrMax + BINCOUNT)%BINCOUNT;
                //calculate turn from bin with max corr in radiant
                turnRad = 2.0*PI-((searchPointIdx*(360.0/BINCOUNT))*PI/180.0);

                //add previous turns to get global rotation
                rotationOffset = rotationOffset + turnRad;
                if (rotationOffset < 0) {
                    rotationOffset = 2.0*PI + rotationOffset;
                } else if (rotationOffset > 2.0*PI) {
                    rotationOffset = rotationOffset - 2.0*PI;
                }

                std::cout<<"orientation now: "<<odom.getOrientation()<<" or before "<<oldPos.getOrientation()<<" turnRad "<<turnRad<<" searchIdx "<<searchPointIdx<<" binsFromDegree "<<binsFromDegree<<endl;

                //rotate scan to same orientation as previous scans
                scan.rotate(rotationOffset);

                // TRANSLATION CORRECTION-----------------------------------------------------------------------------------

                //take the mean of the scan data
                averageScanData(scan, &elementsX[0], &elementsY[0]);

                //calculate x and y histograms
                for (unsigned int i = 0; i < scan.size(); ++i) {
                    if (!isnan(elementsX[i])) {
                        int j = (int) ((elementsX[i] * BINCOUNTDIST/2.0) / MAXLASERDIST) + (BINCOUNTDIST/2.0);
                        histX[j] = histX[j]+1;
                        j = (int) ((elementsY[i] * BINCOUNTDIST/2.0) / MAXLASERDIST) + (BINCOUNTDIST/2.0);
                        histY[j] = histY[j]+1;
                    }
                }

                /*for (int i = 0; i < BINCOUNTDIST; ++i) {
                  cout<<"count: "<<count<<" y "<<i<<" : "<<histY[i]<<endl;
                  }*/

                //get relative movement in x and y direction
                double transXrobot = odom.getX() - oldPos.getX();
                double transYrobot = odom.getY() - oldPos.getY();
                //multiply resulting translation vektor with rotation matrix to translate into global translation vektor
                double transX = cos(rotationOffset) * transXrobot - sin(rotationOffset) * transYrobot;
                double transY = sin(rotationOffset) * transXrobot + cos(rotationOffset) * transYrobot;



                cout<<"transXODOM: "<<transX<<" transYODOM: "<<transY<<endl;

                //calculate index of bin for search in correlation
                int searchIdxX = ((int) ((-transX / ((2*MAXLASERDIST) / BINCOUNTDIST))+BINCOUNTDIST))%BINCOUNTDIST;
                int searchIdxY = ((int) ((-transY / ((2*MAXLASERDIST) / BINCOUNTDIST))+BINCOUNTDIST))%BINCOUNTDIST;

                double corrX[BINCOUNTDIST];
                double corrY[BINCOUNTDIST]; 

                //calculate correlation in x and y histogram
                double globalMaxCorX = 0;
                double globalMaxCorY = 0;
                for (int j = 0; j < BINCOUNTDIST; ++j) {
                    corrX[j] = 0;
                    corrY[j] = 0;
                    for (int i = 0; i < BINCOUNTDIST; ++i) {
                        corrX[j] = corrX[j] + oldHistX[i] * histX[(i+j) % BINCOUNTDIST];
                        corrY[j] = corrY[j] + oldHistY[i] * histY[(i+j) % BINCOUNTDIST];
                    }
                    if (corrX[j] > globalMaxCorX) {
                        globalMaxCorX = corrX[j];
                    }
                    if (corrY[j] > globalMaxCorY) {
                        globalMaxCorY = corrY[j];
                    }

                }

                //calculate how many bins to search around estimated position
                int binsFromTrans = (int) (SEARCHDIST/((2*MAXLASERDIST)/BINCOUNTDIST));
                int corrMaxX = 0;
                double corrMaxValX = 0;
                int corrMaxY = 0;
                double corrMaxValY = 0;
                //search max correalation around estimated position and save relative distance from search origen
                for(int i = 0; i < binsFromTrans; ++i) {
                    if (corrX[(searchIdxX - i + BINCOUNTDIST)%BINCOUNTDIST] > corrMaxValX) {
                        corrMaxX = -i;
                        corrMaxValX = corrX[(searchIdxX - i + BINCOUNTDIST)%BINCOUNTDIST];
                    } 
                    if (corrX[(searchIdxX+i)%BINCOUNTDIST] > corrMaxValX) {
                        corrMaxX = i;
                        corrMaxValX = corrX[(searchIdxX + i)%BINCOUNTDIST];
                    }
                    if (corrY[(searchIdxY - i + BINCOUNTDIST)%BINCOUNTDIST] > corrMaxValY) {
                        corrMaxY = -i;
                        corrMaxValY = corrY[(searchIdxY - i + BINCOUNTDIST)%BINCOUNTDIST];
                    } 
                    if (corrY[(searchIdxY+i)%BINCOUNTDIST] > corrMaxValY) {
                        corrMaxY = i;
                        corrMaxValY = corrY[(searchIdxY + i)%BINCOUNTDIST];
                    }

                }

                //

                int maximumIdxX = (searchIdxX + corrMaxX + BINCOUNTDIST) % BINCOUNTDIST;
                int maximumIdxY = (searchIdxY + corrMaxY + BINCOUNTDIST) % BINCOUNTDIST;

                if (maximumIdxX < BINCOUNTDIST/2) {
                    transX = -(maximumIdxX * 2 * MAXLASERDIST) / BINCOUNTDIST;
                } else {
                    transX = ((BINCOUNTDIST - maximumIdxX) * 2 * MAXLASERDIST) / BINCOUNTDIST;
                }

                if (maximumIdxY < BINCOUNTDIST/2) {
                    transY = -(maximumIdxY * 2 * MAXLASERDIST) / BINCOUNTDIST;
                } else {
                    transY = ((BINCOUNTDIST - maximumIdxY) * 2 * MAXLASERDIST) / BINCOUNTDIST;
                }

                cout<<"transX: "<<transX<<" transY: "<<transY<<" searchidxX: "<<searchIdxX<<" searchidxY: "<<searchIdxY<<" corrMaxX: "<<corrMaxX<<" corrMaxY: "<<corrMaxY<<" binsFromTrans: "<<binsFromTrans<<endl;

                //multiply resulting translation vektor with rotation matrix to translate into global translation vektor
                //double rotatedTransX = cos(rotationOffset) * transX - sin(rotationOffset) * transY;
                //double rotatedTransY = sin(rotationOffset) * transX + cos(rotationOffset) * transY;


                transOffsetX = transOffsetX + transX;
                transOffsetY = transOffsetY + transY;

                //translate scan
                scan.translate(transOffsetX, transOffsetY);
                //scan.translate(1, searchIdxY);

                //cout<<"rottransX: "<<rotatedTransX<<" rottransY: "<<rotatedTransY<<endl;

                //current histograms become old ones
                for (int i = 0; i < BINCOUNTDIST; ++i) {
                    histogramOldX.bins[i] = oldHistX[i];
                    histogramX.bins[i] = histX[i];
                    histogramCorX.bins[i] = (((double) corrX[i])/globalMaxCorX)*290;
                    histogramOldY.bins[i] = oldHistY[i];
                    histogramY.bins[i] = histY[i];
                    histogramCorY.bins[i] = (((double) corrY[i])/globalMaxCorY)*290;
                    oldHistX[i] = histX[i];
                    oldHistY[i] = histY[i];
                }


                //integrate scan
                map.integrate(scan); 
            }

            //update all windows
            scanWindow->update();
            mapWindow->update();
            if (vrWindow)
                vrWindow->update();
            histWindow->update();
            histWindowX->update();
            histWindowY->update();

            //if (count == 0) {
            //waitKey(false);
            // }
            count++;
            //usleep(100000);
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
        if (histWindowY) delete histWindowY;
        if (histWindowX) delete histWindowX;

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
    if (histWindowX) delete histWindowX;
    if (histWindowY) delete histWindowY;

    //
    //
    return EXIT_SUCCESS;
}
