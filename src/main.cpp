# include <Arduino.h>
# include <randLIB.h>
# include <time.h>

int R = PK_5;
int G = PK_6;
int B = PK_7;

void setup() {
    pinMode(R, OUTPUT);
    pinMode(G, OUTPUT);
    pinMode(B, OUTPUT);
    
}

// the loop function runs over and over again forever
/*
void loop() {
    int randseed = time(0);
    randLIB_add_seed(randseed);
    analogWrite(R,randLIB_get_random_in_range(0,255));
    randseed = time(0);
    analogWrite(G,randLIB_get_random_in_range(0,255));
    randseed = time(0);
    analogWrite(B,randLIB_get_random_in_range(0,255));
}*/

void loop() {
    int randseed = time(0);
    randLIB_add_seed(randseed);
    analogWrite(R,200);
    randseed = time(0);
    analogWrite(G,200);
    randseed = time(0);
    analogWrite(B,200);
}
