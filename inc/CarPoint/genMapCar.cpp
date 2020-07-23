#include <iostream>
#include <fstream>
#include <iostream>
#include <math.h>
#include <time.h>
using namespace std;

#define NUMBER_OF_BIT        8
#define NUMBER_OF_BIT_REPEAT 5
#define NUMBER_OF_BIT_REPEAT1 3

int main() {
    ofstream myfile;
    myfile.open ("mapCar.h");

    double N = pow(2,NUMBER_OF_BIT);
    cout<<"NUMBER_OF_BIT:"<<NUMBER_OF_BIT<<endl;
    cout<<"NUMBER_OF_BIT_REPEAT:"<<NUMBER_OF_BIT_REPEAT<<endl;
    cout<<"N:"<<N<<endl;
    // cout<<N<<endl;
    int index = 0;
    int Boss[(int)N];
    for(int i = 1; i < N; i++) {
        int currentBitRepeat    = 1;
        int nextBitRepeat       = 1;
        int currentBitRepeat1   = 0;
        int nextBitRepeat1      = 0;
        int currentStatus       = (i>>0)&0x01;
        int nextStatus;

        for(int j = 0; j < NUMBER_OF_BIT; j++) {
            nextStatus = !!((i>>j)&0x01);
            /*Tìm số bit lặp tối đa*/
            if( currentStatus != nextStatus ) {
                currentStatus = nextStatus;
                currentBitRepeat = 1;
            } else {
                currentBitRepeat++;
            }
            if(nextBitRepeat < currentBitRepeat) {
                nextBitRepeat = currentBitRepeat;
            }
            /*---------------------*/

            /*Tìm số bit 1 lặp tối đa*/
            if( nextStatus == 0) {
                currentBitRepeat1 = 0;
            } else {
                currentBitRepeat1++;
            }
            if(nextBitRepeat1 < currentBitRepeat1) {
                nextBitRepeat1 = currentBitRepeat1;
            }
            /*---------------------*/
        }
        if(nextBitRepeat < NUMBER_OF_BIT_REPEAT && nextBitRepeat1 < NUMBER_OF_BIT_REPEAT1) {
            Boss[index] = i;
            index++;
        }
    }
    cout<<"index:"<<index<<endl;
    int index2 = 0;
    int Boss2[(int)N][2];
    int mapBoss[NUMBER_OF_BIT][2];
    for(int i = 0; i < index; i++) {
        for(int t = 0; t < index; t++) {
            for(int j = 0; j < NUMBER_OF_BIT; j++) {
                mapBoss[j][0] = !!((Boss[i]>>j)&0x01);
                mapBoss[j][1] = !!((Boss[t]>>j)&0x01);
            }
            
            /*       giải thuật       */
            int step = NUMBER_OF_BIT*2;
            int x_point = 0;
            int y_point = 0;

            while(step>0){
                // cout<<step<<endl;
                // cout<<x_point<<"\t"<<y_point<<endl;
                if(x_point < 0 || x_point >= (NUMBER_OF_BIT) || y_point < 0 || y_point > 2){
                    step = -1;
                    // cout<<"Fail x_point:"<<x_point<<" y_point:"<<y_point<<endl;
                } else {
                    if(mapBoss[x_point+1][y_point] == 0 && mapBoss[x_point][y_point] == 0) {// check point ở dưới 
                        x_point++;
                        step --;
                    } else {
                        if(y_point == 1) {
                            if(mapBoss[x_point][y_point-1] == 0) {
                                y_point--;
                                step --;
                            } else {
                                step = -1;
                            }
                        } else {
                            if(mapBoss[x_point][y_point+1] == 0) {
                                y_point++;
                                step --;
                            } else {
                                step = -1;
                            }
                        }
                    }
                }
                // cout<<x_point<<"\t"<<y_point<<endl;
            }
            // cout<<"-----------------------------------"<<endl;
            /*------------------------*/
            if(x_point == (NUMBER_OF_BIT-1)) {
                Boss2[index2][0] = Boss[i];
                Boss2[index2][1] = Boss[t];
                // myfile<<Boss[i]<<"\t"<<Boss[t]<<endl;
                // myfile<<i<<"\t"<<t<<endl;
                // myfile<<endl;
                for(int j = 0; j < NUMBER_OF_BIT; j++) {
                    cout<<mapBoss[j][0]<<"\t"<<mapBoss[j][1]<<endl;
                }            
                cout<<"--------------  "<<index2<<"  -------------------------"<<endl;
                index2 ++;
            }
        }
    }
    cout<<"index2:"<<index2<<endl;
    for(int i = 0; i < index2; i++) {
        cout<<i<<"\t";
        cout<<Boss2[i][0]<<"\t"<<Boss2[i][1]<<endl;
    }
    cout<<"=========================================================================================================="<<endl;

/*------------------Trộn mảng ngẫu nhiên---------------*/    
    srand(time(NULL));
    int minPosition;
    int maxPosition = index2 - 1;
    int swapPosition;
    int i = 0;
    while (i < index2 - 1) {
        minPosition = i + 1;
        swapPosition = rand() % (maxPosition - minPosition + 1) + minPosition;

        swap(Boss2[i][0], Boss2[swapPosition][0]);
        swap(Boss2[i][1], Boss2[swapPosition][1]);
        i++;
    }    
/*-----------------------------------------------------*/    

/*--------------------Print to file--------------------*/
    myfile<<"#define NUMBER_OF_BIT        "<<NUMBER_OF_BIT<<endl;
    myfile<<"#define NUMBER_OF_MAP        "<<index2<<endl;
    if(NUMBER_OF_BIT<=8) {
        myfile<<"uint8";
    } else {
        myfile<<"uint16";
    }
    myfile<<"_t mapCar["<<index2<<"][2]={";
    myfile<<endl;
    for(int i = 0; i < index2; i++) {
        myfile<<"\t{"<<Boss2[i][0]<<","<<Boss2[i][1]<<"},"<<endl;
    }
    myfile<<"};";
/*-----------------------------------------------------*/
    
    // myfile.close();
    return 0;
}
