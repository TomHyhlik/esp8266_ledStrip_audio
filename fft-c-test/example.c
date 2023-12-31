#include "fft.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>



#define BUFFER_EXTENSION_MULTIPLIER     1

#define SPECTR_LOG_SCALE_BAND_cNT        60


void spectrScaledLog_band_GetFreqRange(FFT_PRECISION* frequencyScale, int freqBandCnt);

FFT_PRECISION fft_getMag(FFT_PRECISION* input);



void spectrScaledLog_band_GetFreqRange(FFT_PRECISION* frequencyScale, int freqBandCnt)
{
    FFT_PRECISION start = 1.8;
    FFT_PRECISION stop = 4.3;

    FFT_PRECISION multiplier = (stop - start) / freqBandCnt;

    for (int i = 0; i < freqBandCnt; i++)
    {        
        frequencyScale[i] = 1 * pow(10, (start + ((FFT_PRECISION)(multiplier * i))));
        // printf("%d\t%.3f\n", (int)i, frequencyScale[i]);
    }
}

// double myinput[] = {-46.415965,0.282605,0.864665,-45.280774,10.061582,-41.385651,-39.671262,12.372653,-52.162170,-9.623845,1.720746,-43.067932,20.972252,-31.023979,-28.358142,24.031321,-43.887774,2.468109,5.647977,-38.006465,26.476860,-22.233009,-10.552406,37.589391,-37.815412,12.111664,-7.578532,-38.528760,20.795822,-12.450218,-16.974449,47.446251,-31.190554,7.301648,19.668579,-35.336494,16.103109,-22.275607,-25.130272,22.212029,-41.222572,1.485189,17.596563,-55.032094,20.104090,-18.292109,-24.524689,26.668866,-33.373833,5.916595,28.192520,-31.788190,28.846105,-4.115105,-16.618729,39.241791,-34.948667,22.715251,28.902690,-25.960286,42.853673,15.518506,-6.353696,51.864942,-19.384384,8.103053,35.595576,-33.035914,34.206072,5.439123,-9.666125,54.832776,-10.375023,18.944740,58.961233,-11.192322,43.670654,7.982254,-21.995544,46.842257,0.000318,40.772756,65.576553,-4.433632,50.176938,18.797239,-12.568792,46.103477,-17.856598,7.448832,44.784228,-19.941966,36.404928,16.753515,-9.864489,47.044436,-27.328491,-3.248851,40.277799,-25.363604,11.474609,3.914197,-41.154226,32.891591,-26.288668,-1.875877,48.484484,-32.454173,15.852928,5.746206,-25.915464,38.654645,-23.430824,-19.990603,21.826744,-45.220057,18.576940,6.626765,-39.312363,24.802844,-20.995776,-3.171921,32.100677,-48.059464,12.690544,9.752909,-21.642049,53.148905,-6.307920,3.302892,37.979126,-38.512230,13.353666,15.881538,-22.687912,44.521014,-14.763196,-18.836657,32.409032,-34.133593,22.912343,17.550786,-37.135124,24.065971,-15.635490,-35.341581,12.799899,-53.564072,-25.639216,2.683004,-19.400915,49.425443,11.727969,19.033750,80.087980,-1.045863,38.237254,43.969154,1.064301,65.183640,21.920522,11.235555,60.619990,-8.823395,30.923208,40.570577,-25.051753,27.515411,-10.735512,-21.104495,28.784434,-41.003863,-4.912694,15.355428,-43.718656,19.551913,-10.920842,-5.978266,52.378337,-13.328552,13.154984,18.518448,-34.051895,37.246386,-8.090019,-12.575785,40.681521,-49.111366,-9.686152,17.093976,-41.597048,9.923617,-27.789434,-38.321177,19.824346,-42.098681,1.262665,18.038114,-43.762207,14.958064,-20.561854,-42.537053,35.128911,-28.742790,-1.929601,21.390915,-42.608897,16.744932,-0.623703,-27.315776,24.534225,-36.282857,-10.386467,9.693782,-48.558871,-3.890673,-40.157636,-62.048594,9.010951,-54.229101,-39.836248,-6.662051,-84.135056,-21.822929,-41.985512,-66.702843,10.742505,-55.323283,-22.440275,8.095423,-48.565229,13.182958,-2.005577,-38.991928,30.845006,-32.186190,7.397652,36.278725,-27.514458,34.200668,18.015862,-13.909976,49.827258,-13.920148,3.998121,32.347361,-30.599276,20.641327,-0.055313,-27.271271,40.429115,-16.369184,12.341181,52.383105,-13.769150,41.768710,22.632599,-30.579567,44.179281,-14.057477,8.863767,67.112923,-1.794179,37.413597,32.869339,-4.006386,56.609790,-2.902985,13.156573,49.615542,-11.911074,26.848157,33.691088,-2.102852,58.803876,9.898822,4.957517,47.643026,-29.180209,26.833852,22.257487,-26.871363,37.805875,-0.218391,1.846313,66.873868,-17.312368,25.957108,20.101229,-31.045596,42.196274,0.813802,-7.719994,48.498154,-41.023890,-5.147934,2.657255,-32.107035,25.117238,-18.117905,-15.700658,28.138796,-43.406487,5.072912,10.566076,-41.272163,10.882378,-25.448481,-20.255725,19.951185,-51.342010,-2.920787,14.897982,-36.532084,15.478134,-18.488566,-32.329877,22.871971,-28.896650,4.973412,21.251043,-28.929710,26.874860,-14.388084,-34.326553,49.588839,-39.396286,-1.839956,9.858449,-39.127032,8.243243,-23.813248,-39.858500,14.677684,-42.195002,-11.308988,-6.255786,-57.982763,1.871745,-22.627195,-31.092326,34.682592,-48.300107,-9.133021,19.385974,-43.005308,11.169434,-15.960375,-42.442004,17.490387,-39.827347,5.285263,34.611066,-34.598986,12.613297,-6.626129,-30.102412,37.151019,-28.550466,-10.553996,27.754784,-37.648837,22.910754,-5.552928,-28.400421,33.926328,-37.970225,-10.390917,28.295517,-30.443509,25.691668,-3.534635,-34.104665,22.509893,-50.583839,-18.421173,12.723605,-60.606321,0.001272,-9.888649,-44.763883,20.279566,-18.502235,-6.258965,34.648577,-40.878614,9.251912,3.039042,-48.882484,25.225004,-31.362851,-25.142670,83.399455,-39.374034,4.219055,2.778053,-45.107841,29.787381,-30.047417,-11.651039,24.534543,-38.967133,16.685804,3.166199,-53.181966,14.637629,-52.677472,-37.567139,12.630145,-60.827255,-9.538651,-3.444672,-57.663600,13.731003,-42.881330,-29.815356,17.221769,-42.310397,12.823105,6.537120,-39.544106,16.271909,-27.211825,-27.566274,21.525701,-44.719696,-14.265378,1.042366,-29.559771,36.459605,-12.202263,-11.713982,-26.619275,-36.392212,18.916766,44.828097,1.855850,54.998716,-1.395543,3.601074,54.933866,-9.555817,22.675196,25.330544,-20.048459,49.401283,15.728951,14.028231,65.910975,7.864952,25.566419,58.126767,0.866254,58.543205,32.668749,15.324275,60.544650,-3.195445,44.601440,59.116681,-6.856600,58.135351,21.785736,6.715457,63.676834,-1.118660,30.093829,50.807317,-4.231453,57.969093,46.013832,19.787470,72.167079,19.539833,35.184542,54.315885,18.755277,52.913666,26.078542,5.328178};

void fourier_example()
{
    printf("---------- wrapped fourier transform example ----------\n");

    FFT_PRECISION spectrumScaleLog_freq[SPECTR_LOG_SCALE_BAND_cNT];
    FFT_PRECISION spectrumScaleLog_mag[SPECTR_LOG_SCALE_BAND_cNT];

    // Construct input signal
    FFT_PRECISION sample_rate = 200000; // signal sampling rate
    FFT_PRECISION signal_length_ms = 50;
    FFT_PRECISION f = 1010;    // frequency of the artifical signal

    /* Get logarithmic scale of the spectrum */
    spectrScaledLog_band_GetFreqRange(spectrumScaleLog_freq, SPECTR_LOG_SCALE_BAND_cNT);

    // for (int i = 0; i < SPECTR_LOG_SCALE_BAND_cNT; i++)
    // {
    //     printf("%d\t%.3f\n", (int)i, spectrumScaleLog_freq[i]);
    // }

    int input_cnt = BUFFER_EXTENSION_MULTIPLIER * signal_length_ms / 1000 * sample_rate; // n has to be greater than 1

    FFT_PRECISION input[input_cnt];  
    
    memset(input, 0, sizeof(input));

    // FFT_PRECISION * input = myinput;
    // int n = sizeof(myinput)/sizeof(double);

    // printf("\nTime data START =======\n");
    // for(int i = 0; i < n; i ++) 
    //     printf("%lf,",i/sample_rate);
    // printf("\nTime data END =======\n");

    /* Set the input data */
    for (int i = 0; i < sizeof(input)/sizeof(FFT_PRECISION)/BUFFER_EXTENSION_MULTIPLIER; i++) 
    {
         //2*PI*f*t
        // input[i] = 1 * sin(2 * M_PI * f * i/sample_rate);
        input[i] = 1 * sin(2 * M_PI * f * i/sample_rate) +
                    1 * sin(2 * M_PI * f/2 * i/sample_rate) +
                    1 * sin(2 * M_PI * f/10 * i/sample_rate) +
                    1 * sin(2 * M_PI * f*2 * i/sample_rate);
    }

    // printf("\nRaw data START =======\n");
    // for(int i = 0; i < sizeof(input)/sizeof(FFT_PRECISION); i ++) 
    // {
    //     printf("Raw_sample[%d] =\t%lf\n,", i, input[i]);
    // }
    // printf("\nRaw data END =======\n");


    // Initialize fourier transformer
    FFTTransformer* transformer = create_fft_transformer(input_cnt, FFT_SCALED_OUTPUT);


    // Transform signal
    fft_forward(transformer, input);

    /* Scale spectrum to log - use top value of the spectrum */
    {
        FFT_PRECISION cnt = 0;
        FFT_PRECISION mag = 0;

        for (int band = 0; band < SPECTR_LOG_SCALE_BAND_cNT; band++)
        {
            for (int i = 0; i < sizeof(input)/sizeof(FFT_PRECISION); i += 2)
            {
                /* Calc FREQUENCY */
                FFT_PRECISION freq = i / 2 * sample_rate / input_cnt;

                if (freq < spectrumScaleLog_freq[band])
                {
                    continue;
                }
                else if (freq >= spectrumScaleLog_freq[band + 1])
                {
                    if (cnt == 0)
                    {
                        if (i >= 2)
                        {
                            spectrumScaleLog_mag[band] = fft_getMag(&input[i-2]);
                        }
                        break;
                    }
                    else
                    {
                        spectrumScaleLog_mag[band] = mag;
                    }
                    mag = 0;
                    cnt = 0;
                    break;
                }

                FFT_PRECISION mag_current = fft_getMag(&input[i]);

                if (mag_current > mag)
                {
                    mag = mag_current;
                }

                cnt += 1;

                if (band >= SPECTR_LOG_SCALE_BAND_cNT) break; // rm later
            }
        }
    }


    // /* Scale spectrum to log - averaging */
    // {
    //     FFT_PRECISION sum = 0;
    //     FFT_PRECISION cnt = 0;

    //     for (int band = 0; band < SPECTR_LOG_SCALE_BAND_cNT; band++)
    //     {
    //         for (int i = 0; i < sizeof(input)/sizeof(FFT_PRECISION); i += 2)
    //         {
    //             /* Calc FREQUENCY */
    //             FFT_PRECISION freq = i / 2 * sample_rate / input_cnt;

    //             if (freq < spectrumScaleLog_freq[band])
    //             {
    //                 continue;
    //             }
    //             else if (freq >= spectrumScaleLog_freq[band + 1])
    //             {
    //                 if (cnt == 0)
    //                 {
    //                     i -= 2;
    //                     freq = i / 2 * sample_rate / input_cnt;
    //                     spectrumScaleLog_mag[band] = fft_getMag(&input[i]);
    //                     break;
    //                 }
    //                 else
    //                 {
    //                     spectrumScaleLog_mag[band] = sum / cnt;
    //                 }
    //                 sum = 0;
    //                 cnt = 0;
    //                 break;
    //             }

    //             FFT_PRECISION mag = fft_getMag(&input[i]);

    //             // printf("%d\tFreq:\t%.1f, MAG:\t%d\n", i, freq, (int)(mag*1000));

    //             sum += mag;
    //             cnt += 1;

    //             if (band >= SPECTR_LOG_SCALE_BAND_cNT) break; // rm later
    //         }
    //     }
    // }





//    Print output
    for (int i = 0; i < sizeof(input)/sizeof(FFT_PRECISION); i += 2)
    {
        FFT_PRECISION freq = i / 2 * sample_rate / input_cnt;
        FFT_PRECISION cos_comp = input[i];
        FFT_PRECISION sin_comp = input[i+1];
        FFT_PRECISION mag = 1000 * sqrt((cos_comp * cos_comp) + (sin_comp * sin_comp));
        // printf("Freq:%f,COS:%f\n", freq, cos_comp);
        // printf("Freq:%f,SIN:%f\n", freq, sin_comp);

        /* Print */
        if (freq >= 50 && freq <= 2500)
        {
            // printf("Freq:\t%f, MAG:\t%f\n", freq, mag);
            printf("%d\t%.1f\t%d\n", i, freq, (int)(mag));
        }
    }

    printf("---------- Spectrum Log Scaled ----------\n");
    for (int i = 0; i < SPECTR_LOG_SCALE_BAND_cNT; i++)
    {
        printf("%d\t%.1f\t%.1f\n", (int)i, spectrumScaleLog_freq[i], spectrumScaleLog_mag[i]);
     }
    
    free_fft_transformer(transformer);
    printf("---------- wrapped fourier transform example end ----------\n");
}

/* Calc MAGNITUDE */
FFT_PRECISION fft_getMag(FFT_PRECISION* input)
{
    FFT_PRECISION cos_comp = input[0];
    FFT_PRECISION sin_comp = input[1];
    FFT_PRECISION mag = 1000 * sqrt((cos_comp * cos_comp) + (sin_comp * sin_comp));
    return mag;           
}



int main() 
{   
    printf("========= FFT example =========\n\n");
    //fftpack_fourier_forward();
    fourier_example();

    printf("========= Done. =========\n\n");
    return 0;
}

