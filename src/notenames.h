/*!
    \file    notenames.h
*/

/*
    Copyright (c) 2020 Rafael Bachmann

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef NOTENAMES_H

#include <stdint.h>

char *note_names[] = {
    "B",
    "C",
    "C#",
    "D",
    "D#",
    "E",
    "F",
    "F#",
    "G",
    "G#",
    "A",
    "A#",
    "B",
    "C",
    "C#",
    "D",
    "D#",
    "E",
    "F",
    "F#",
    "G",
    "G#",
    "A",
    "A#",
    "B",
    "C",
    "C#",
    "D",
    "D#",
    "E",
    "F",
    "F#",
    "G",
    "G#",
    "A",
    "A#",
    "B",
    "C",
    "C#",
    "D",
    "D#",
    "E",
    "F",
    "F#",
    "G",
    "G#",
    "A",
    "A#",
    "B",
    "C",
};

char* get_note_name(uint32_t freq) {
    if (freq < 127) {
        return note_names[0];
    }
    if (freq < 135) {
        return note_names[1];
    }
    if (freq < 143) {
        return note_names[2];
    }
    if (freq < 151) {
        return note_names[3];
    }
    if (freq < 160) {
        return note_names[4];
    }
    if (freq < 170) {
        return note_names[5];
    }
    if (freq < 180) {
        return note_names[6];
    }
    if (freq < 191) {
        return note_names[7];
    }
    if (freq < 202) {
        return note_names[8];
    }
    if (freq < 214) {
        return note_names[9];
    }
    if (freq < 227) {
        return note_names[10];
    }
    if (freq < 240) {
        return note_names[11];
    }
    if (freq < 254) {
        return note_names[12];
    }
    if (freq < 269) {
        return note_names[13];
    }
    if (freq < 285) {
        return note_names[14];
    }
    if (freq < 302) {
        return note_names[15];
    }
    if (freq < 320) {
        return note_names[16];
    }
    if (freq < 339) {
        return note_names[17];
    }
    if (freq < 360) {
        return note_names[18];
    }
    if (freq < 381) {
        return note_names[19];
    }
    if (freq < 404) {
        return note_names[20];
    }
    if (freq < 428) {
        return note_names[21];
    }
    if (freq < 453) {
        return note_names[22];
    }
    if (freq < 480) {
        return note_names[23];
    }
    if (freq < 509) {
        return note_names[24];
    }
    if (freq < 539) {
        return note_names[25];
    }
    if (freq < 571) {
        return note_names[26];
    }
    if (freq < 605) {
        return note_names[27];
    }
    if (freq < 641) {
        return note_names[28];
    }
    if (freq < 679) {
        return note_names[29];
    }
    if (freq < 719) {
        return note_names[30];
    }
    if (freq < 762) {
        return note_names[31];
    }
    if (freq < 807) {
        return note_names[32];
    }
    if (freq < 855) {
        return note_names[33];
    }
    if (freq < 906) {
        return note_names[34];
    }
    if (freq < 960) {
        return note_names[35];
    }
    if (freq < 1017) {
        return note_names[36];
    }
    if (freq < 1078) {
        return note_names[37];
    }
    if (freq < 1142) {
        return note_names[38];
    }
    if (freq < 1210) {
        return note_names[39];
    }
    if (freq < 1282) {
        return note_names[40];
    }
    if (freq < 1358) {
        return note_names[41];
    }
    if (freq < 1438) {
        return note_names[42];
    }
    if (freq < 1524) {
        return note_names[43];
    }
    if (freq < 1615) {
        return note_names[44];
    }
    if (freq < 1711) {
        return note_names[45];
    }
    if (freq < 1812) {
        return note_names[46];
    }
    if (freq < 1920) {
        return note_names[47];
    }
    if (freq < 2034) {
        return note_names[0];
    }
    return "";
};

#endif // NOTENAMES_H