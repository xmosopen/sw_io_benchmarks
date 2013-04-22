#include <platform.h>
#include <xs1.h>
#include <xclib.h>
#include <print.h>

#define PORT_COUNT 1

out port alertPort = XS1_PORT_4F;

clock clk = XS1_CLKBLK_1;
out port outPort = XS1_PORT_1A;

buffered in port:1 inPort[4] = {XS1_PORT_1B, XS1_PORT_1C, XS1_PORT_1D, XS1_PORT_1E};

#define TS_CT_1_WAIT (1<<15)
#define TS_CT_2_WAIT 400
#define ITERATIONS 0xffff

#define PAD_DELAY (2)
void test_CT_InOut(){
  unsigned ts_out, ts_in[PORT_COUNT], t;
  unsigned i, j, portDiff;
  unsigned up_total = 0, down_total = 0;
  unsigned up_min = 0x7fffffff, up_max = 0;
  unsigned down_min = 0x7fffffff, down_max = 0;
  stop_clock(clk);
  configure_out_port(outPort, clk, 0);
  for(i=0;i<PORT_COUNT;i++){
	  configure_in_port(inPort[i], clk);
  }
  start_clock(clk);
  outPort <: 0 @ ts_out;
  ts_out += TS_CT_1_WAIT;
  outPort @ ts_out <: 0 ;
  outPort <: 0 @ ts_out;
  ts_out += TS_CT_1_WAIT;

  for(j=0;j<ITERATIONS;j++) {
//while(1){

		outPort @ ts_out <: 1;
		par {
			select {case inPort[0] when pinseq(1) :> void @ ts_in[0]:break;}
		//	select {case inPort[1] when pinseq(1) :> void @ ts_in[1]:break;}
		//	select {case inPort[2] when pinseq(1) :> void @ ts_in[2]:break;}
		//	select {case inPort[3] when pinseq(1) :> void @ ts_in[3]:break;}
		}
		//printintln(ts_out);
		for(i=0;i<PORT_COUNT;i++) {
			//printintln(ts_in[i]);
			portDiff = 0xffff&(ts_in[i] - ts_out) - PAD_DELAY;
			//TODO add overflow detection
			up_total += portDiff;
			if (portDiff < up_min) up_min = portDiff;
			if (portDiff > up_max) up_max = portDiff;
		}

		crc32(t, ts_out, 0x1EDC6F41);
		t = t & 0x7fff;
		outPort <: 1 @ ts_out;
		ts_out = ts_out + TS_CT_1_WAIT + t;

		outPort @ ts_out <: 0;
		par {
			select {case inPort[0] when pinseq(0) :> void @ ts_in[0]:break;}
		//	select {case inPort[1] when pinseq(0) :> void @ ts_in[1]:break;}
		//	select {case inPort[2] when pinseq(0) :> void @ ts_in[2]:break;}
		//	select {case inPort[3] when pinseq(0) :> void @ ts_in[3]:break;}
		}

		//printintln(ts_out);
		for(i=0;i<PORT_COUNT;i++) {
			//printintln(ts_in[i]);
			portDiff = 0xffff&(ts_in[i] - ts_out) - PAD_DELAY;
			//TODO add overflow detection
			down_total += portDiff;
			if (portDiff < down_min) down_min = portDiff;
			if (portDiff > down_max) down_max = portDiff;
		}

		crc32(t, ts_out, 0x1EDC6F41);
		t = t & 0x7fff;
		outPort <: 0 @ ts_out;
		ts_out = ts_out + TS_CT_1_WAIT + t;
	}
  printuint(up_min);
  printstr("\t");
  printuint(up_max);
  printstr("\t");
  printuint(up_total);
  printstr("\t");

  printuint(down_min);
  printstr("\t");
  printuint(down_max);
  printstr("\t");
  printuintln(down_total);

}

/*
void test_CT_InOut2(){
	unsigned ts_out, ts_in, t;
	  unsigned up_total = 0,down_total = 0, i, diff;
	  unsigned up_min = 0xffff, up_max = 0;
	  unsigned down_min = 0xffff, down_max = 0;
	  stop_clock(clk);
	  configure_out_port(outPort, clk, 0);
	  configure_in_port(inPort, clk);
	  start_clock(clk);

	  inPort :> t;
	  if(!t){
		  outPort <: 1 @ ts_out;
		  ts_out += TS_CT_1_WAIT;
		  outPort @ ts_out <: 0;
		  ts_out += TS_CT_1_WAIT;
		  outPort @ ts_out <: 0;
	  }

	  outPort <: 0 @ ts_out;
	  ts_out += TS_CT_1_WAIT;
	  for(i=0;i<ITERATIONS; i++){
		  outPort @ ts_out <: 1;
		  inPort when pinseq(0) :> void @ ts_in;

		  diff = 0xffff&(ts_in - ts_out);
		  up_total += diff;
		  if (diff < up_min) up_min = diff;
		  else if (diff > up_max) up_max = diff;

		  outPort <: 1 @ ts_out;
		  crc32(t, ts_out, 0x1EDC6F41);
		  t = t & 0x7fff;
		  ts_out = ts_out + TS_CT_1_WAIT + t;
		  outPort @ ts_out <: 0 ;
		  crc32(t, ts_out, 0x1EDC6F41);
		  t = t & 0x7fff;
		  ts_out = ts_out + TS_CT_1_WAIT + t;

		  outPort @ ts_out <: 1;
		  inPort when pinseq(1) :> void @ ts_in;

		  diff = 0xffff&(ts_in - ts_out);
		  down_total += diff;
		  if (diff < down_min) down_min = diff;
		  else if (diff > down_max) down_max = diff;

		  outPort <: 1 @ ts_out;
		  crc32(t, ts_out, 0x1EDC6F41);
		  t = t & 0x7fff;
		  ts_out = ts_out + TS_CT_1_WAIT + t;
		  outPort @ ts_out<: 0 ;
		  crc32(t, ts_out, 0x1EDC6F41);
		  t = t & 0x7fff;
		  ts_out = ts_out + TS_CT_1_WAIT + t;
	  }

}
*/

int mainHarness(void) {
	unsigned i;
	for (i = 0; i < 8; i++) {
		test_CT_InOut();
	}
	return 0;
}

int main(){


/*
	timer t;
	unsigned x;
	unsigned y=0;
	t:> x;
	alertPort <: 0xf;
	while(1){
		x += 10000000;
		t when timerafter(x) :> void;
		outPort <: y;

		//inPort[1] when pinseq(y) :> int;
		y = ~y;
	}
*/
	mainHarness();

	return 0;
}
