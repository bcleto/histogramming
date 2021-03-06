// testbench for partial histogramming v1

#include <stdint.h>
#include "hls_stream.h"
//#include "ap_int.h"
#include "hls_math.h"


const int fifo_depth=128;
void part_hist_v1(
		ap_uint<115> data_in,
		ap_uint<1>   input_valid,
		ap_uint<32>  accumulation,
		hls::stream<ap_uint<16> > &frequency_out,
		hls::stream<ap_uint<16> >  &mode_out
		);


ap_uint<3> mini_module[8];
ap_uint<95> mini_module_register;
ap_uint<4> sub_module;
ap_uint<14> tdc_value;
ap_uint<115> one_channel_data;
hls::stream<ap_uint<16> > frequency;
hls::stream<ap_uint<16> > mode;

ap_uint<32> bb;
ap_uint<32> bc;
ap_uint<1>   input_valid;
ap_uint<32>  accumulation;


int main(){

	for(int i=0;i<=0;i++){
		//sub_module[i]=((rand())%16);
		sub_module=0;
		tdc_value = ((rand())%2);
		//tdc_value = 0;

		printf("submodule[%i]: %d \n",i,(unsigned int)sub_module);

			for(int t=0; t<=7; t++){
				mini_module[t]=((rand())%8);
				printf("mm_[%i]: %d \n",t,(unsigned int)mini_module[t]);
				one_channel_data=(ap_uint<115> (sub_module)<<111)+(ap_uint<115> (tdc_value)<<96) + (ap_uint<115> (mini_module[7])<<90)+
						(ap_uint<115> (mini_module[6])<<78)+(ap_uint<115> (mini_module[5])<<66)+(ap_uint<115> (mini_module[4])<<54)+
						(ap_uint<115> (mini_module[3])<<42)+(ap_uint<115> (mini_module[2])<<30)+(ap_uint<115> (mini_module[1])<<18)+
						(ap_uint<115> (mini_module[0])<<6);


			}

				part_hist_v1(	one_channel_data,
								1,
								0,
								frequency,
								mode
								);
				//printf("fifo1:%i \n",(unsigned int) data_out_fifo);
				//printf("fifo2: %i \n", (unsigned int)pixel_addr_fifo);

		}


	for(int iOut=0;iOut<fifo_depth;iOut++){
		ap_uint<16> outdata_dataout;
		ap_uint<16> pixel_addr_fifo2;
		frequency.read(outdata_dataout);
		mode.read(pixel_addr_fifo2);

		printf("freq[%i]:%i \n",iOut,(unsigned int) outdata_dataout);
		printf("mode[%i]: %i \n",iOut, (unsigned int)pixel_addr_fifo2);
	}
	return 0;
}
