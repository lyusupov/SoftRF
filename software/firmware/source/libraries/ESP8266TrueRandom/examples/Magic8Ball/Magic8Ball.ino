/*
 * A magic 8 ball.
 *
 * Press the reset button to see into the future.
 *
 * View the answer to your question in the Serial Monitor, at 19200 baud.
 *
 * Press the Arduino reset button to ask another question.
 *
 */

#include "ESP8266TrueRandom.h"

char* answers[20] = {
  "As I see it, yes",
  "It is certain",
  "It is decidedly so",
  "Mostly likely",
  "Outlook good",
  "Signs point to yes",
  "Without a doubt",
  "Yes",
  "Yes - definitely",
  "You may rely on it",
  "Reply hazy, try again",
  "Ask again later",
  "Better not tell you now",
  "Cannot predict now",
  "Concentrate and ask again",
  "Don't count on it",
  "My reply is no",
  "My sources say no",
  "Outlook not so good",
  "Very doubtful"
};

int answerNumber;

void setup() {
  Serial.begin(9600);

  Serial.print("The answer is ");
  
  // Dramatic pause
  delay(1000);
  Serial.print(". ");
  delay(1000);
  Serial.print(". ");
  delay(1000);
  Serial.print(". ");
  delay(1000);

  answerNumber = ESP8266TrueRandom.random(20);
  Serial.println( answers[answerNumber] );
}

void loop() {
  ; // Do nothing
}
