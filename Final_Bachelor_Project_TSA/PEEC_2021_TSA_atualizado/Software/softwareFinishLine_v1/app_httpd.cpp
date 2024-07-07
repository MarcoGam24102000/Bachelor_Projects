// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "camera_index.h"
#include "Arduino.h"

#include "fb_gfx.h"
#include "fd_forward.h"
#include "fr_forward.h"

#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 7

#define FACE_COLOR_WHITE  0x00FFFFFF
#define FACE_COLOR_BLACK  0x00000000
#define FACE_COLOR_RED    0x000000FF
#define FACE_COLOR_GREEN  0x0000FF00
#define FACE_COLOR_BLUE   0x00FF0000
#define FACE_COLOR_YELLOW (FACE_COLOR_RED | FACE_COLOR_GREEN)
#define FACE_COLOR_CYAN   (FACE_COLOR_BLUE | FACE_COLOR_GREEN)
#define FACE_COLOR_PURPLE (FACE_COLOR_BLUE | FACE_COLOR_RED)



//hw_timer_t * timer;
//int ledPin = 13;
//extern int ledPin;
extern bool readytoGo;
extern bool start_race;
//extern void endTimer(hw_timer_t * timer);
//extern void startTimer(hw_timer_t * timer);
//void vsync_intr_disable();
//volatile byte state = LOW;
bool TLtimerFlag = false;
extern bool tl_is_running;
int intr_alloc_flags;
//volatile int8_t blink_n = 0;

extern bool checkIfDetected;

volatile bool wait_and_presentTime = false;

void waitAndPresentTime();

volatile int64_t time2 = 0;

extern bool num_serie_changed;

//----------------------------------------------------------------------//
//volatile bool start_race = false;
//----------------------------------------------------------------------//

//==================================================================//
//Variáveis Adicionadas
//
extern int8_t buffB[120*160];           
extern int8_t frame_ant [19200];
int64_t fr_stop = 0;

extern bool camera;
//
//==================================================================//

int codePropMethod = 0;


typedef struct {
        size_t size; //number of values used for filtering
        size_t index; //current value index
        size_t count; //value count
        int sum;
        int * values; //array to be filled with values
} ra_filter_t;

typedef struct {
        httpd_req_t *req;
        size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static ra_filter_t ra_filter;
httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

static mtmn_config_t mtmn_config = {0};
static int8_t detection_enabled = 0;
static int8_t recognition_enabled = 0;
static int8_t is_enrolling = 0;
static face_id_list id_list = {0};

static ra_filter_t * ra_filter_init(ra_filter_t * filter, size_t sample_size){
    memset(filter, 0, sizeof(ra_filter_t));

    filter->values = (int *)malloc(sample_size * sizeof(int));
    if(!filter->values){
        return NULL;
    }
    memset(filter->values, 0, sample_size * sizeof(int));

    filter->size = sample_size;
    return filter;
}



//==================================================================//
//
// Codigo XPTO
//
//==================================================================//




static int ra_filter_run(ra_filter_t * filter, int value){
    if(!filter->values){
        return value;
    }
    filter->sum -= filter->values[filter->index];
    filter->values[filter->index] = value;
    filter->sum += filter->values[filter->index];
    filter->index++;
    filter->index = filter->index % filter->size;
    if (filter->count < filter->size) {
        filter->count++;
    }
    return filter->sum / filter->count;
}

static void rgb_print(dl_matrix3du_t *image_matrix, uint32_t color, const char * str){
    fb_data_t fb;
    fb.width = image_matrix->w;
    fb.height = image_matrix->h;
    fb.data = image_matrix->item;
    fb.bytes_per_pixel = 3;
    fb.format = FB_BGR888;
    fb_gfx_print(&fb, (fb.width - (strlen(str) * 14)) / 2, 10, color, str);
}

static int rgb_printf(dl_matrix3du_t *image_matrix, uint32_t color, const char *format, ...){
    char loc_buf[64];
    char * temp = loc_buf;
    int len;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    len = vsnprintf(loc_buf, sizeof(loc_buf), format, arg);
    va_end(copy);
    if(len >= sizeof(loc_buf)){
        temp = (char*)malloc(len+1);
        if(temp == NULL) {
            return 0;
        }
    }
    vsnprintf(temp, len+1, format, arg);
    va_end(arg);
    rgb_print(image_matrix, color, temp);
    if(len > 64){
        free(temp);
    }
    return len;
}

static void draw_face_boxes(dl_matrix3du_t *image_matrix, box_array_t *boxes, int face_id){
    int x, y, w, h, i;
    uint32_t color = FACE_COLOR_YELLOW;
    if(face_id < 0){
        color = FACE_COLOR_RED;
    } else if(face_id > 0){
        color = FACE_COLOR_GREEN;
    }
    fb_data_t fb;
    fb.width = image_matrix->w;
    fb.height = image_matrix->h;
    fb.data = image_matrix->item;
    fb.bytes_per_pixel = 3;
    fb.format = FB_BGR888;
    for (i = 0; i < boxes->len; i++){
        // rectangle box
        x = (int)boxes->box[i].box_p[0];
        y = (int)boxes->box[i].box_p[1];
        w = (int)boxes->box[i].box_p[2] - x + 1;
        h = (int)boxes->box[i].box_p[3] - y + 1;
        fb_gfx_drawFastHLine(&fb, x, y, w, color);
        fb_gfx_drawFastHLine(&fb, x, y+h-1, w, color);
        fb_gfx_drawFastVLine(&fb, x, y, h, color);
        fb_gfx_drawFastVLine(&fb, x+w-1, y, h, color);
#if 0
        // landmark
        int x0, y0, j;
        for (j = 0; j < 10; j+=2) {
            x0 = (int)boxes->landmark[i].landmark_p[j];
            y0 = (int)boxes->landmark[i].landmark_p[j+1];
            fb_gfx_fillRect(&fb, x0, y0, 3, 3, color);
        }
#endif
    }
}

static int run_face_recognition(dl_matrix3du_t *image_matrix, box_array_t *net_boxes){
    dl_matrix3du_t *aligned_face = NULL;
    int matched_id = 0;

    aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
    if(!aligned_face){
        Serial.println("Could not allocate face recognition buffer");
        return matched_id;
    }
    if (align_face(net_boxes, image_matrix, aligned_face) == ESP_OK){
        if (is_enrolling == 1){
            int8_t left_sample_face = enroll_face(&id_list, aligned_face);

            if(left_sample_face == (ENROLL_CONFIRM_TIMES - 1)){
                Serial.printf("Enrolling Face ID: %d\n", id_list.tail);
            }
            Serial.printf("Enrolling Face ID: %d sample %d\n", id_list.tail, ENROLL_CONFIRM_TIMES - left_sample_face);
            rgb_printf(image_matrix, FACE_COLOR_CYAN, "ID[%u] Sample[%u]", id_list.tail, ENROLL_CONFIRM_TIMES - left_sample_face);
            if (left_sample_face == 0){
                is_enrolling = 0;
                Serial.printf("Enrolled Face ID: %d\n", id_list.tail);
            }
        } else {
            matched_id = recognize_face(&id_list, aligned_face);
            if (matched_id >= 0) {
                Serial.printf("Match Face ID: %u\n", matched_id);
                rgb_printf(image_matrix, FACE_COLOR_GREEN, "Hello Subject %u", matched_id);
            } else {
                Serial.println("No Match Found");
                rgb_print(image_matrix, FACE_COLOR_RED, "Intruder Alert!");
                matched_id = -1;
            }
        }
    } else {
        Serial.println("Face Not Aligned");
        //rgb_print(image_matrix, FACE_COLOR_YELLOW, "Human Detected");
    }

    dl_matrix3du_free(aligned_face);
    return matched_id;
}

static size_t jpg_encode_stream(void * arg, size_t index, const void* data, size_t len){
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if(!index){
        j->len = 0;
    }
    if(httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK){
        return 0;
    }
    j->len += len;
    return len;
}

//static esp_err_t capture_handler(httpd_req_t *req){
//    camera_fb_t * fb = NULL;
//    esp_err_t res = ESP_OK;
//    int64_t fr_start = esp_timer_get_time();
//
//    fb = esp_camera_fb_get();
//     if (!fb) {
//        Serial.println("Camera capture failed");
//        httpd_resp_send_500(req);
//        return ESP_FAIL;
//    }
//
//
//    //==================================================================//
//    // Cálculo do Erro
//    //
//    int64_t erro=0;
//    int64_t err_vector[120*160];
//    
//    for (int16_t i = 0; i < fb->len; i++){
//      erro += abs(buffB[i] - fb->buf[i]);
//      }
//    //
//    //==================================================================//
//    for (int16_t i = 0; i < fb->len; i++){
//      buffB[i] = fb->buf[i];
//      }
//
////    //**************************************************************************************************
////    //Código para detetar, a partir do sensor de luminosidade e, por meio de muitas formulações matemáticas,
////    //qual o tipo de luz que se trata
////    if(luz == luzSolar){
////      if(erro >= limit1){      //limit1 = Erro Médio inerente à diferença de duas imagens, à luz do Sol
////        //começa a contar o tempo
////        //fr_start = esp_timer_get_time();        
////      }
////    }else if(luz == luzArtificial){
////      if(erro >= limit2){      //limit2 = Erro Médio inerente à diferença de duas imagens, com luz artificial
////        //começa a contar o tempo
////        //fr_start = esp_timer_get_time();        
////      }
////    }else{
////      Serial.println("ERRO");  // É uma mensagem que não irá aparecer. Coloquei-a aqui com um intuito meramente informativo
////                               // Ao invés, não deverá acontecer literalmente nada quando se detetar que a luz não é solar nem artificial.
////    }
//    //**************************************************************************************************
//    Serial.printf("Image Capture  Size:%u W:%u H:%u F:%u   E:%u  \n",fb->len, fb->width, fb->height, fb->format, erro); //Alteracoes SF
//    
//                                                                                                               
//                                                                                                               //ERRO
//    
//    //==================================================================//
//    // Vetor do erro
//    //
//    for (int16_t i = 0; i < fb->len; i++){
//      //fb->buf[i] = err_vector[i];
//      err_vector[i] = buffB[i] - fb->buf[i]; //Diferença dos dois vetores de imagens (vetor de zeros se as imagens forem iguais)
//      Serial.printf("%u", err_vector[i]);
//      if(err_vector[i] != 0){
//        Serial.println("Não totalmente iguais");
//        break;
//      }
//      fb->buf[i] = err_vector[i];
//    }
//    //
//    //==================================================================//
//
//    httpd_resp_set_type(req, "image/jpeg");
//    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
//    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
//
//    size_t out_len, out_width, out_height;
//    uint8_t * out_buf;
//    bool s;
//    bool detected = false;
//    int face_id = 0;
//    if(!detection_enabled || fb->width > 400){
//        size_t fb_len = 0;
//        if(fb->format == PIXFORMAT_JPEG){
//            fb_len = fb->len;
//            res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
//        } else {
//            jpg_chunking_t jchunk = {req, 0};
//            res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk)?ESP_OK:ESP_FAIL;
//            httpd_resp_send_chunk(req, NULL, 0);
//            fb_len = jchunk.len;
//        }
//        esp_camera_fb_return(fb);
//        int64_t fr_end = esp_timer_get_time();
//        Serial.printf("JPG: %uB %ums\n", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start)/1000));
//        return res;
//    }
//
//    dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
//    if (!image_matrix) {
//        esp_camera_fb_return(fb);
//        Serial.println("dl_matrix3du_alloc failed");
//        httpd_resp_send_500(req);
//        return ESP_FAIL;
//    }
//
//    
//
//    out_buf = image_matrix->item;
//    out_len = fb->width * fb->height * 3;
//    out_width = fb->width;
//    out_height = fb->height;
//
//   s = fmt2rgb888(fb->buf, fb->len, fb->format, out_buf);
//
//    esp_camera_fb_return(fb);
//    if(!s){
//        dl_matrix3du_free(image_matrix);
//        Serial.println("to rgb888 failed");
//        httpd_resp_send_500(req);
//        return ESP_FAIL;
//    }
//
//    box_array_t *net_boxes = face_detect(image_matrix, &mtmn_config);
//
//    if (net_boxes){
//        detected = true;
//        if(recognition_enabled){
//            face_id = run_face_recognition(image_matrix, net_boxes);
//        }
//        draw_face_boxes(image_matrix, net_boxes, face_id);
//        free(net_boxes->score);
//        free(net_boxes->box);
//        free(net_boxes->landmark);
//        free(net_boxes);
//    }
//
//    jpg_chunking_t jchunk = {req, 0};
//    s = fmt2jpg_cb(out_buf, out_len, out_width, out_height, PIXFORMAT_RGB888, 90, jpg_encode_stream, &jchunk);
//    dl_matrix3du_free(image_matrix);
//    if(!s){
//        Serial.println("JPEG compression failed");
//        return ESP_FAIL;
//    }
//
//    int64_t fr_end = esp_timer_get_time();
//    Serial.printf("FACE: %uB %ums %s%d\n", (uint32_t)(jchunk.len), (uint32_t)((fr_end - fr_start)/1000), detected?"DETECTED ":"", face_id);
//    return res;
//}

//static esp_err_t stream_handler(httpd_req_t *req){
//
//
//    if(readytoGo){
//      //configGPIO();
//      //init_GPIO();
//      readytoGo = false;
//    }
//    camera_fb_t * fb = NULL;
//    esp_err_t res = ESP_OK;
//    size_t _jpg_buf_len = 0;
//    uint8_t * _jpg_buf = NULL;
//    char * part_buf[64];
//    dl_matrix3du_t *image_matrix = NULL;
//    bool detected = false;
//    int face_id = 0;
//    int64_t fr_start = 0;
//    int64_t fr_ready = 0;
//    int64_t fr_face = 0;
//    int64_t fr_recognize = 0;
//    int64_t fr_encode = 0;
//
//    //===================================================
//    //Variável usada para contagem do tempo
//    //
//    int64_t lastimage = 0;
//    //    
//    //===================================================
//
//    //===================================================
//    //
//    //Flag relacionada com o momento entre os dois instantes      
//      //int8_t flag_break = 0;
//    //
//    //=================================================== 
//
//    static int64_t last_frame = 0;
////    if(!last_frame) {
////        last_frame = esp_timer_get_time();
////    }
//
//    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
//    if(res != ESP_OK){
//        return res;
//    }
//
//    //=============================================================
//    //
////    int8_t ev=0;                             //Declaração da flag
//    //
//    //=============================================================
//
////    int16_t count_tent = 0;
//
//    int16_t count=0; 
//    int16_t count_afterStart = 0;
//
//    int8_t start=0; //Flag que indica se o tempo começou ou não a contar
////
//    int64_t time_dif_flag = 0;
////    
//
//    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
//
//    while(true){
//        detected = false;
//        face_id = 0;
//
//        
//        fr_start = esp_timer_get_time();
//        fb = esp_camera_fb_get();       
//
//        //**************************************************************************************************
//   //=======================================================================================================================
//      
//      //Declaraçao de variáveis
//      
//      int16_t i = 0;           
//      int16_t soma_lastElements = 0;
//      //int8_t frame_ant [19200];     
//      int64_t mae=0;      
//      int64_t soma_maes=0;
//      int32_t soma_maes_p_pixel=0;
//      //int16_t mae=0;
//      //int16_t count_tent = 0;
//      
//  
//
//      
//
//      //Cálculo da soma das diferenças dos píxeis
//      
//      //if(count_tent > 0){
//        for (int16_t i = 0; i < fb->len; i++){
//          if(frame_ant[i] >= fb->buf[i]){
//            mae += (frame_ant[i] - fb->buf[i]);
//          }else{
//            mae += (fb->buf[i] - frame_ant[i]);
//          }
//        }         
//      //}
//       
//
//      //count_tent++;
//
//      //===========================================
//      for (int16_t i = 0; i < fb->len; i++){
//         frame_ant[i] = fb->buf[i];
//      }   
//      //===========================================
//     
//
//     //Serial.printf("Image Capture1  Size:%u W:%u H:%u F:%u MAE:%u \n",fb->len, fb->width, fb->height, fb->format, mae); //Alteracoes SF
//     
//     //Serial.printf("Image Capture2  Size:%u W:%u H:%u F:%u \n",fb->len, fb->width, fb->height, fb->format); //Alteracoes SF
//
//     
//     //===========================================
//     //Condição para definição do limiar
//     
//       if ((mae/19200)> 0){
//         //Serial.printf("Imagem Diferente!!");
//         Serial.printf("\n%u",(int32_t)(mae/19200));
//       }
//     //
//     //===========================================
//     
//     //======================================================
//     // Média por píxel
//     //
//     count++;
//     soma_maes += mae;
//     //Serial.printf("MEDIA:%u \n", (soma_maes/count));
//     soma_maes_p_pixel += (int32_t)(mae/19200);
//     //Serial.printf("MEDIA POR PÍXEL:");
//     //Serial.printf("%u \n", (soma_maes_p_pixel/count));
//     //
//     //======================================================
//      
//     last_frame = esp_timer_get_time();
//      
//     
//     //lastimage = last_frame;
//     //======================================================
//     // Condição para definição do limiar
//     //
//
//
//     //=========================================================
//     // Para garantir um intervalo entre os dois instantes, no qual as imagens possui muito menos diferença entre si 
//     //(permitindo assim contar o tempo corretamente)
//     // Elaborar um gráfico (MatLab) que explicite esta ideia (para o Relatório)
//     //
////     if((soma_maes_p_pixel/count)<10){               
////      flag_break = 1;
////      //Serial.printf("\r\nFlag break: %d", flag_break);
////     }
//     //=========================================================
//     
//     
////     if(flag_break == 0){
////       if((soma_maes_p_pixel/count)>10){
////       //lastimage = last_frame;
////       //lastimage = esp_timer_get_time();
////        ev++;                                     //Incrementação da flag
////        if(ev == 1){
////          if(count > 1){                            //Para não considerar a primeira imagem
////            lastimage = esp_timer_get_time();
////          }else{} 
////          Serial.println("Start");
////        } else if(ev > 1){         
////            fr_stop = esp_timer_get_time();
////            Serial.println("Stop");
////        }
////       }
////     }else{
////      //Serial.println("It´s not working");
////     }
////     
////      int64_t delta_t = fr_stop - lastimage;
////      int64_t delta_t_s = delta_t/1000;
////
////      if(lastimage != 0 && fr_stop != 0){
////        Serial.printf("\nTIME DIF: %u", delta_t_s);
////      }
////      
////        
////      //Serial.printf("Valor da Flag: %u", ev);
////
//
//     //=========================================================
//     // Para garantir um intervalo entre os dois instantes, no qual as imagens possui muito menos diferença entre si 
//     //(permitindo assim contar o tempo corretamente)
//     // Elaborar um gráfico (MatLab) que explicite esta ideia (para o Relatório)
//     //
////     if((mae/19200)<15){               
////      flag_break = 1;
////      //Serial.printf("\r\nFlag break: %d", flag_break);
////     }
//     //=========================================================
//
////      if(flag_break == 1){
////       if((mae/19200)>15){
////       //lastimage = last_frame;
////       //lastimage = esp_timer_get_time();
////        ev++;                                     //Incrementação da flag
////        if(ev == 1){
////          if(count > 1){                            //Para não considerar a primeira imagem
////            lastimage = esp_timer_get_time();
////            Serial.println("Start");
////            start = 1;
////          }else{
////            Serial.println("ERRO");
////          }           
////        } else if(ev > 1){
////            if(start == 1){       
////              fr_stop = esp_timer_get_time();
////              ev = 0;                              // ev - Número de vezes em que é ultrapassado o limiar
////              time_dif_flag = 1;
////              Serial.println("Stop");
////              start=0;
////            }
////        } else{ Serial.println("Condição Impossível");}
////       }
////     }else{
////      //Serial.println("It´s not working");
////     }
////     
////      int64_t delta_t = abs(fr_stop - lastimage);
////      int64_t delta_t_s = delta_t/1000;
////
////      //if(lastimage != 0 && fr_stop != 0){
////
////      if(time_dif_flag == 1){
////        Serial.printf("\t\t\tTIME DIF: %u", delta_t_s);
////        time_dif_flag = 0;
////      }
////
//
////     if(count>1){
////      if(((mae/19200)<15) && (start==1)){
////        count_afterStart++;
////      }
////      else if((mae/19200)>15){
////        if(count_afterStart==0){
////          lastimage = esp_timer_get_time();
////          Serial.println("Start");        
////          start=1;
////        }
////        count_afterStart==0
////        if(count_afterStart>10 && start==1){
////          fr_stop = esp_timer_get_time();
////          Serial.println("Stop");
////          time_dif_flag = 1;
////          start=0;
////          count_afterStart=0;
////          count=0;        
////        }
////      }
////      
////     }
//
//
//     if(start_race){
//       //lastimage = esp_timer_get_time();
//       Serial.println("Start");        
//       start=1;      
//     }
//     
//
//     if(start == 1 && checkIfDetected){
//        checkIfDetected = false;
//        if(((mae/19200)<15)){
//          count_afterStart++;
//        }else if(((mae/19200)>15)){
//          if(count_afterStart>10){
//            //fr_stop = esp_timer_get_time();
//            Serial.println("Stop");
//            wait_and_presentTime = true;
//            time_dif_flag = 1;
//            start=0;
//            count_afterStart=0;
//            start_race = false;
//          }
//        }
//     }
//
//     int64_t delta_t = abs(fr_stop - lastimage);
//     int64_t delta_t_s = delta_t/1000;
//
//     if(time_dif_flag == 1){
//       Serial.printf("\t\t\tTIME DIF: %u", delta_t_s);
//       time_dif_flag = 0;
//     }
//
//        
//      //Serial.printf("Valor da Flag: %u", ev);
//
//
//     //======================================================================================================================================
//     // Cálculo do Vetor do Erro
//     //
//     int64_t err_vector_x[19200];
//     
//     for (int32_t i = 0; i < fb->len; i++){
//      //fb->buf[i] = err_vector[i];
//      err_vector_x[i] = abs(frame_ant[i] - fb->buf[i]);
////      if(frame_ant[i] >= fb->buf[i]){
////        err_vector_x[i] = frame_ant[i] - fb->buf[i]; //Diferença dos dois vetores de imagens (vetor de zeros se as imagens forem iguais) 
////      }else{
////        err_vector_x[i] = fb->buf[i] - frame_ant[i]; //Diferença dos dois vetores de imagens (vetor de zeros se as imagens forem iguais)
//      //}
//      //Serial.printf("%u", err_vector_x[i]);
//      if(i>((fb->len)-10)){
//        soma_lastElements += err_vector_x[i];
//      }
////      if(i<11){
////        soma_lastElements += err_vector_x[i];
////      }
//      if(err_vector_x[i] != 0){
//        //Serial.println("Não totalmente iguais");
//        break;
//      }
//      //fb->buf[i] = err_vector_x[i];
//    }
//
//    //Serial.printf("\n%u -> %u", soma_lastElements, (soma_lastElements/2));
//    //======================================================================================================================================
//
//    //=======================================================================================================================
//    
//        if (!fb) {
//            Serial.println("Camera capture failed");
//            res = ESP_FAIL;
//        } else {
//            //fr_start = esp_timer_get_time();
//            fr_ready = fr_start;
//            fr_face = fr_start;
//            fr_encode = fr_start;
//            fr_recognize = fr_start;
//            if(!detection_enabled || fb->width > 400){
//                if(fb->format != PIXFORMAT_JPEG){
//                    bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
//                    esp_camera_fb_return(fb);
//                    fb = NULL;
//                    if(!jpeg_converted){
//                        Serial.println("JPEG compression failed");
//                        res = ESP_FAIL;
//                    }
//                } else {
//                    _jpg_buf_len = fb->len;
//                    _jpg_buf = fb->buf;
//                }
//            } else {
//
//                image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
//
//                if (!image_matrix) {
//                    Serial.println("dl_matrix3du_alloc failed");
//                    res = ESP_FAIL;
//                } else {
//                    if(!fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item)){
//                        Serial.println("fmt2rgb888 failed");
//                        res = ESP_FAIL;
//                    } else {
//                        fr_ready = esp_timer_get_time();
//                        box_array_t *net_boxes = NULL;
//                        if(detection_enabled){
//                            net_boxes = face_detect(image_matrix, &mtmn_config);
//                        }
//                        fr_face = esp_timer_get_time();
//                        fr_recognize = fr_face;
//                        if (net_boxes || fb->format != PIXFORMAT_JPEG){
//                            if(net_boxes){
//                                detected = true;
//                                if(recognition_enabled){
//                                    face_id = run_face_recognition(image_matrix, net_boxes);
//                                }
//                                fr_recognize = esp_timer_get_time();
//                                draw_face_boxes(image_matrix, net_boxes, face_id);
//                                free(net_boxes->score);
//                                free(net_boxes->box);
//                                free(net_boxes->landmark);
//                                free(net_boxes);
//                            }
//                            if(!fmt2jpg(image_matrix->item, fb->width*fb->height*3, fb->width, fb->height, PIXFORMAT_RGB888, 90, &_jpg_buf, &_jpg_buf_len)){
//                                Serial.println("fmt2jpg failed");
//                                res = ESP_FAIL;
//                            }
//                            esp_camera_fb_return(fb);
//                            fb = NULL;
//                        } else {
//                            _jpg_buf = fb->buf;
//                            _jpg_buf_len = fb->len;
//                        }
//                        fr_encode = esp_timer_get_time();
//                    }
//                    dl_matrix3du_free(image_matrix);
//                }
//            }
//        }
//        if(res == ESP_OK){
//            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
//            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
//        }
//        if(res == ESP_OK){
//            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
//        }
//        if(res == ESP_OK){
//            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
//        }
//        if(fb){
//            esp_camera_fb_return(fb);
//            fb = NULL;
//            _jpg_buf = NULL;
//        } else if(_jpg_buf){
//            free(_jpg_buf);
//            _jpg_buf = NULL;
//        }
//        if(res != ESP_OK){
//            break;
//        }
//        int64_t fr_end = esp_timer_get_time();
//
//        int64_t ready_time = (fr_ready - fr_start)/1000;
//        int64_t face_time = (fr_face - fr_ready)/1000;
//        int64_t recognize_time = (fr_recognize - fr_face)/1000;
//        int64_t encode_time = (fr_encode - fr_recognize)/1000;
//        int64_t process_time = (fr_encode - fr_start)/1000;
//
//        //Serial.printf("%u\t%u\t%u\t%u\t%u", ready_time, face_time, recognize_time, encode_time, process_time); 
//        
//        int64_t frame_time = fr_end - last_frame;
//        //int64_t delta_t = fr_end - lastimage;
//        //int64_t delta_t_s = delta_t/1000;
//        //Serial.printf("\nTIME DIF: %u", delta_t_s);
//        last_frame = fr_end;
//        frame_time /= 1000;
//        uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
//        //Serial.printf("MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps), %u+%u+%u+%u=%u %s%d\n",
//        //    (uint32_t)(_jpg_buf_len),
//        //    (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time,
//        //    avg_frame_time, 1000.0 / avg_frame_time,
//        //    (uint32_t)ready_time, (uint32_t)face_time, (uint32_t)recognize_time, (uint32_t)encode_time, (uint32_t)process_time,
//        //    (detected)?"DETECTED ":"", face_id
//        //);
//    }
//
//    last_frame = 0;
//    return res;
//}

int countingMAEvalues = 0;

int findingBestMAEValue(int arr[], int len){  

  int countVector[len];
  int count = 0;
  int myNumber = 0;
  int numberMostFreq = 0;
  int i_mostFreq = 0;  
  int uniqueValues = 0;  
  int posMostFreq = 0;    
  int higherValue = 0;  
  //int bestValue = 0;
  int codePropMethod = 0;

  for(int j=0; j<len; j++){
    myNumber = arr[j];
    for (int i=0; i<len; i++){
      if (arr[i] == myNumber){
        count++;
      }

      if(i==len-1){
        countVector[j] = count;
        count = 0;
      }
    }
  }

//  Serial.println("\n\nVetor: \n");
//  for(int i=0; i<len; i++){
//    Serial.print(countVector[i]);
//  }

  for(int i=0; i<len; i++){  //countVector em vez de arr
    if(countVector[i] >= posMostFreq){
      posMostFreq = countVector[i];

//      Serial.printf("\nConsidering %d higher\n", countVector[i]);
      i_mostFreq = i;
    }

    if(i==len-1){
      numberMostFreq = arr[i_mostFreq];
    }
  }

  for(int i=0; i<len; i++){  // Acabar fluxograma
    if(countVector[i] == 1){  // Acabar fluxograma
      uniqueValues++;         // Acabar fluxograma
    }
  }
  // Acabar fluxograma
  if(uniqueValues != len){  //se todos os valores não forem diferentes
    return numberMostFreq;
    codePropMethod = 1;
  }else{    // Acabar fluxograma                //se os valores forem todos diferentes, encontra o valor mais alto
    for(int i = 0; i<len; i++){  // Fluxograma feito
      if(arr[i] >= higherValue){ // Fluxograma feito
        higherValue = arr[i]; // Fluxograma feito
      } // Fluxograma feito
    } // Fluxograma feito

    codePropMethod = 2;

    return higherValue;
  }

  //  Serial.printf("\nNumber most frequent: %d", numberMostFreq); 
}

const int lenMAE_array = 20;

int32_t THR_ARR_MAE[lenMAE_array];

int bestValue = 0;

int mostCommonValue = 0;
int valueHigher = 0;

int Lim_MAE = 15;


void stream_handler(){

    ra_filter_init(&ra_filter, 20);


    mtmn_config.type = FAST;
    mtmn_config.min_face = 80;
    mtmn_config.pyramid = 0.707;
    mtmn_config.pyramid_times = 4;
    mtmn_config.p_threshold.score = 0.6;
    mtmn_config.p_threshold.nms = 0.7;
    mtmn_config.p_threshold.candidate_number = 20;
    mtmn_config.r_threshold.score = 0.7;
    mtmn_config.r_threshold.nms = 0.7;
    mtmn_config.r_threshold.candidate_number = 10;
    mtmn_config.o_threshold.score = 0.7;
    mtmn_config.o_threshold.nms = 0.7;
    mtmn_config.o_threshold.candidate_number = 1;
    
    face_id_init(&id_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES); 



    Serial.println("\nstream_handler\n");

    if(readytoGo){
      //configGPIO();
      //init_GPIO();
      readytoGo = false;
    }
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t * _jpg_buf = NULL;
    char * part_buf[64];
    dl_matrix3du_t *image_matrix = NULL;
    bool detected = false;
    int face_id = 0;
    int64_t fr_start = 0;
    int64_t fr_ready = 0;
    int64_t fr_face = 0;
    int64_t fr_recognize = 0;
    int64_t fr_encode = 0;

    //===================================================
    //Variável usada para contagem do tempo
    //
    int64_t lastimage = 0;
    //    
    //===================================================

    //===================================================
    //
    //Flag relacionada com o momento entre os dois instantes      
      //int8_t flag_break = 0;
    //
    //=================================================== 

    static int64_t last_frame = 0;
//    if(!last_frame) {
//        last_frame = esp_timer_get_time();
//    }

    //res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
//    if(res != ESP_OK){
//        return res;
//    }

    //=============================================================
    //
//    int8_t ev=0;                             //Declaração da flag
    //
    //=============================================================

//    int16_t count_tent = 0;

    int16_t count=0; 
    int16_t count_afterStart = 0;

    int8_t start=0; //Flag que indica se o tempo começou ou não a contar
//
    int64_t time_dif_flag = 0;
//    

    //httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    while(1){   //true

//        if(!camera){
//          break;
//        }

        if(num_serie_changed){
          num_serie_changed = false;
          break;
        }



        
        detected = false;
        face_id = 0;

        
        fr_start = esp_timer_get_time();
        fb = esp_camera_fb_get();       

        //**************************************************************************************************
   //=======================================================================================================================
      
      //Declaraçao de variáveis
      
      int16_t i = 0;           
      int16_t soma_lastElements = 0;
      //int8_t frame_ant [19200];     
      int64_t mae=0;      
      int64_t soma_maes=0;
      int32_t soma_maes_p_pixel=0;
      //int16_t mae=0;
      //int16_t count_tent = 0;
      
  

      

      //Cálculo da soma das diferenças dos píxeis
      
      //if(count_tent > 0){
        for (int16_t i = 0; i < fb->len; i++){
          if(frame_ant[i] >= fb->buf[i]){
            mae += (frame_ant[i] - fb->buf[i]);
          }else{
            mae += (fb->buf[i] - frame_ant[i]);
          }
        }         
      //}
       

      //count_tent++;

      //===========================================
      for (int16_t i = 0; i < fb->len; i++){
         frame_ant[i] = fb->buf[i];
      }   
      //===========================================
     

     //Serial.printf("Image Capture1  Size:%u W:%u H:%u F:%u MAE:%u \n",fb->len, fb->width, fb->height, fb->format, mae); //Alteracoes SF
     
     //Serial.printf("Image Capture2  Size:%u W:%u H:%u F:%u \n",fb->len, fb->width, fb->height, fb->format); //Alteracoes SF

     
     //===========================================
     //Condição para definição do limiar
     
       if ((mae/19200)> Lim_MAE){
         //Serial.printf("Imagem Diferente!!");
         //Serial.printf("\n%u",(int32_t)(mae/19200));
       }

       //Serial.printf("\nValue: %u",(int32_t)(mae/19200));       

       THR_ARR_MAE[countingMAEvalues] = mae/19200;

       if(countingMAEvalues == lenMAE_array-1){
          countingMAEvalues = 0;
          bestValue = findingBestMAEValue(THR_ARR_MAE, lenMAE_array);
       }else if(countingMAEvalues < lenMAE_array-1){
          countingMAEvalues++;
       }     

      if(codePropMethod == 1){
        mostCommonValue = bestValue;
        Serial.printf("\nMost Common Value: %d\n", mostCommonValue);  
      }else if(codePropMethod == 2){
        valueHigher = bestValue;
        Serial.printf("\nHigher Value: %d\n", valueHigher); 
      }   

      Lim_MAE = bestValue;   //Redefinição do limiar, tendo em conta os dados armazenados


//       if (((mae/19200)/10)> 15){
//         //Serial.printf("Imagem Diferente!!");
//         Serial.printf("\n%u",(int32_t)(mae/19200));
//       }
     //
     //===========================================
     
     //======================================================
     // Média por píxel
     //
     count++;
     soma_maes += mae;
     //Serial.printf("MEDIA:%u \n", (soma_maes/count));
     soma_maes_p_pixel += (int32_t)(mae/19200);
     //Serial.printf("MEDIA POR PÍXEL:");
     //Serial.printf("%u \n", (soma_maes_p_pixel/count));
     //
     //======================================================
      
     last_frame = esp_timer_get_time();
      
     
     //lastimage = last_frame;
     //======================================================
     // Condição para definição do limiar
     //


     //=========================================================
     // Para garantir um intervalo entre os dois instantes, no qual as imagens possui muito menos diferença entre si 
     //(permitindo assim contar o tempo corretamente)
     // Elaborar um gráfico (MatLab) que explicite esta ideia (para o Relatório)
     //
//     if((soma_maes_p_pixel/count)<10){               
//      flag_break = 1;
//      //Serial.printf("\r\nFlag break: %d", flag_break);
//     }
     //=========================================================
     
     
//     if(flag_break == 0){
//       if((soma_maes_p_pixel/count)>10){
//       //lastimage = last_frame;
//       //lastimage = esp_timer_get_time();
//        ev++;                                     //Incrementação da flag
//        if(ev == 1){
//          if(count > 1){                            //Para não considerar a primeira imagem
//            lastimage = esp_timer_get_time();
//          }else{} 
//          Serial.println("Start");
//        } else if(ev > 1){         
//            fr_stop = esp_timer_get_time();
//            Serial.println("Stop");
//        }
//       }
//     }else{
//      //Serial.println("It´s not working");
//     }
//     
//      int64_t delta_t = fr_stop - lastimage;
//      int64_t delta_t_s = delta_t/1000;
//
//      if(lastimage != 0 && fr_stop != 0){
//        Serial.printf("\nTIME DIF: %u", delta_t_s);
//      }
//      
//        
//      //Serial.printf("Valor da Flag: %u", ev);
//

     //=========================================================
     // Para garantir um intervalo entre os dois instantes, no qual as imagens possui muito menos diferença entre si 
     //(permitindo assim contar o tempo corretamente)
     // Elaborar um gráfico (MatLab) que explicite esta ideia (para o Relatório)
     //
//     if((mae/19200)<15){               
//      flag_break = 1;
//      //Serial.printf("\r\nFlag break: %d", flag_break);
//     }
     //=========================================================

//      if(flag_break == 1){
//       if((mae/19200)>15){
//       //lastimage = last_frame;
//       //lastimage = esp_timer_get_time();
//        ev++;                                     //Incrementação da flag
//        if(ev == 1){
//          if(count > 1){                            //Para não considerar a primeira imagem
//            lastimage = esp_timer_get_time();
//            Serial.println("Start");
//            start = 1;
//          }else{
//            Serial.println("ERRO");
//          }           
//        } else if(ev > 1){
//            if(start == 1){       
//              fr_stop = esp_timer_get_time();
//              ev = 0;                              // ev - Número de vezes em que é ultrapassado o limiar
//              time_dif_flag = 1;
//              Serial.println("Stop");
//              start=0;
//            }
//        } else{ Serial.println("Condição Impossível");}
//       }
//     }else{
//      //Serial.println("It´s not working");
//     }
//     
//      int64_t delta_t = abs(fr_stop - lastimage);
//      int64_t delta_t_s = delta_t/1000;
//
//      //if(lastimage != 0 && fr_stop != 0){
//
//      if(time_dif_flag == 1){
//        Serial.printf("\t\t\tTIME DIF: %u", delta_t_s);
//        time_dif_flag = 0;
//      }
//

//     if(count>1){
//      if(((mae/19200)<15) && (start==1)){
//        count_afterStart++;
//      }
//      else if((mae/19200)>15){
//        if(count_afterStart==0){
//          lastimage = esp_timer_get_time();
//          Serial.println("Start");        
//          start=1;
//        }
//        count_afterStart==0
//        if(count_afterStart>10 && start==1){
//          fr_stop = esp_timer_get_time();
//          Serial.println("Stop");
//          time_dif_flag = 1;
//          start=0;
//          count_afterStart=0;
//          count=0;        
//        }
//      }
//      
//     }


     if(start_race){
       //lastimage = esp_timer_get_time();
       //Serial.println("Start");        
       start=1;      
     }

//     Serial.println("\nstart: ");
//     Serial.print(start);
//     Serial.print("\ncheckIfDetected: ");
//     Serial.print(checkIfDetected);
//     Serial.println("\n");

     if(start == 1 && checkIfDetected){     
        //Serial.println("\nstart=1\n");        
        if(((mae/19200)<Lim_MAE)){
          count_afterStart++;
        }else if((mae/19200)>Lim_MAE){     
          if(count_afterStart>10){
            //fr_stop = esp_timer_get_time();
            //time2 = esp_timer_get_time();
            //Serial.println("Stop");
            //wait_and_presentTime = true;

            checkIfDetected = false;

            waitAndPresentTime();
            
            time_dif_flag = 1;
            start=0;
            count_afterStart=0;
            start_race = false;
          }
        }
     }

     int64_t delta_t = abs(fr_stop - lastimage);
     int64_t delta_t_s = delta_t/1000;

     if(time_dif_flag == 1){
       //Serial.printf("\t\t\tTIME DIF: %u", delta_t_s);
       time_dif_flag = 0;
     }

        
      //Serial.printf("Valor da Flag: %u", ev);


     //======================================================================================================================================
     // Cálculo do Vetor do Erro
     //
     int64_t err_vector_x[19200];
     
     for (int32_t i = 0; i < fb->len; i++){
      //fb->buf[i] = err_vector[i];
      err_vector_x[i] = abs(frame_ant[i] - fb->buf[i]);
//      if(frame_ant[i] >= fb->buf[i]){
//        err_vector_x[i] = frame_ant[i] - fb->buf[i]; //Diferença dos dois vetores de imagens (vetor de zeros se as imagens forem iguais) 
//      }else{
//        err_vector_x[i] = fb->buf[i] - frame_ant[i]; //Diferença dos dois vetores de imagens (vetor de zeros se as imagens forem iguais)
      //}
      //Serial.printf("%u", err_vector_x[i]);
      if(i>((fb->len)-10)){
        soma_lastElements += err_vector_x[i];
      }
//      if(i<11){
//        soma_lastElements += err_vector_x[i];
//      }
      if(err_vector_x[i] != 0){
        //Serial.println("Não totalmente iguais");
        break;
      }
      //fb->buf[i] = err_vector_x[i];
    }

    //Serial.printf("\n%u -> %u", soma_lastElements, (soma_lastElements/2));
    //======================================================================================================================================

    //=======================================================================================================================
    
        if (!fb) {
            Serial.println("Camera capture failed");
            //res = ESP_FAIL;
        } else {
            //fr_start = esp_timer_get_time();
            fr_ready = fr_start;
            fr_face = fr_start;
            fr_encode = fr_start;
            fr_recognize = fr_start;
            if(!detection_enabled || fb->width > 400){
                if(fb->format != PIXFORMAT_JPEG){
                    bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                    esp_camera_fb_return(fb);
                    fb = NULL;
                    if(!jpeg_converted){
                        Serial.println("JPEG compression failed");
                        //res = ESP_FAIL;
                    }
                } else {
                    _jpg_buf_len = fb->len;
                    _jpg_buf = fb->buf;
                }
            } else {

                image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);

                if (!image_matrix) {
                    Serial.println("dl_matrix3du_alloc failed");
                    //res = ESP_FAIL;
                } else {
                    if(!fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item)){
                        Serial.println("fmt2rgb888 failed");
                        //res = ESP_FAIL;
                    } else {
                        fr_ready = esp_timer_get_time();
                        box_array_t *net_boxes = NULL;
                        if(detection_enabled){
                            net_boxes = face_detect(image_matrix, &mtmn_config);
                        }
                        fr_face = esp_timer_get_time();
                        fr_recognize = fr_face;
                        if (net_boxes || fb->format != PIXFORMAT_JPEG){
                            if(net_boxes){
                                detected = true;
                                if(recognition_enabled){
                                    face_id = run_face_recognition(image_matrix, net_boxes);
                                }
                                fr_recognize = esp_timer_get_time();
                                draw_face_boxes(image_matrix, net_boxes, face_id);
                                free(net_boxes->score);
                                free(net_boxes->box);
                                free(net_boxes->landmark);
                                free(net_boxes);
                            }
                            if(!fmt2jpg(image_matrix->item, fb->width*fb->height*3, fb->width, fb->height, PIXFORMAT_RGB888, 90, &_jpg_buf, &_jpg_buf_len)){
                                Serial.println("fmt2jpg failed");
                                //res = ESP_FAIL;
                            }
                            esp_camera_fb_return(fb);
                            fb = NULL;
                        } else {
                            _jpg_buf = fb->buf;
                            _jpg_buf_len = fb->len;
                        }
                        fr_encode = esp_timer_get_time();
                    }
                    dl_matrix3du_free(image_matrix);
                }
            }
        }
//        if(res == ESP_OK){
//            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
//            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
//        }
//        if(res == ESP_OK){
//            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
//        }
//        if(res == ESP_OK){
//            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
//        }
        if(fb){
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        } else if(_jpg_buf){
            free(_jpg_buf);
            _jpg_buf = NULL;
        }
//        if(res != ESP_OK){
//            break;
//        }
        int64_t fr_end = esp_timer_get_time();

        int64_t ready_time = (fr_ready - fr_start)/1000;
        int64_t face_time = (fr_face - fr_ready)/1000;
        int64_t recognize_time = (fr_recognize - fr_face)/1000;
        int64_t encode_time = (fr_encode - fr_recognize)/1000;
        int64_t process_time = (fr_encode - fr_start)/1000;

        //Serial.printf("%u\t%u\t%u\t%u\t%u", ready_time, face_time, recognize_time, encode_time, process_time); 
        
        int64_t frame_time = fr_end - last_frame;
        //int64_t delta_t = fr_end - lastimage;
        //int64_t delta_t_s = delta_t/1000;
        //Serial.printf("\nTIME DIF: %u", delta_t_s);
        last_frame = fr_end;
        frame_time /= 1000;
        uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
        //Serial.printf("MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps), %u+%u+%u+%u=%u %s%d\n",
        //    (uint32_t)(_jpg_buf_len),
        //    (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time,
        //    avg_frame_time, 1000.0 / avg_frame_time,
        //    (uint32_t)ready_time, (uint32_t)face_time, (uint32_t)recognize_time, (uint32_t)encode_time, (uint32_t)process_time,
        //    (detected)?"DETECTED ":"", face_id
        //);
    }

    last_frame = 0;
    //return res;
}

static esp_err_t cmd_handler(httpd_req_t *req){
    char*  buf;
    size_t buf_len;
    char variable[32] = {0,};
    char value[32] = {0,};

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char*)malloc(buf_len);
        if(!buf){
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK &&
                httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK) {
            } else {
                free(buf);
                httpd_resp_send_404(req);
                return ESP_FAIL;
            }
        } else {
            free(buf);
            httpd_resp_send_404(req);
            return ESP_FAIL;
        }
        free(buf);
    } else {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    int val = atoi(value);
    sensor_t * s = esp_camera_sensor_get();
    int res = 0;

    if(!strcmp(variable, "framesize")) {
        if(s->pixformat == PIXFORMAT_JPEG) res = s->set_framesize(s, (framesize_t)val);
    }
    else if(!strcmp(variable, "quality")) res = s->set_quality(s, val);
    else if(!strcmp(variable, "contrast")) res = s->set_contrast(s, val);
    else if(!strcmp(variable, "brightness")) res = s->set_brightness(s, val);
    else if(!strcmp(variable, "saturation")) res = s->set_saturation(s, val);
    else if(!strcmp(variable, "gainceiling")) res = s->set_gainceiling(s, (gainceiling_t)val);
    else if(!strcmp(variable, "colorbar")) res = s->set_colorbar(s, val);
    else if(!strcmp(variable, "awb")) res = s->set_whitebal(s, val);
    else if(!strcmp(variable, "agc")) res = s->set_gain_ctrl(s, val);
    else if(!strcmp(variable, "aec")) res = s->set_exposure_ctrl(s, val);
    else if(!strcmp(variable, "hmirror")) res = s->set_hmirror(s, val);
    else if(!strcmp(variable, "vflip")) res = s->set_vflip(s, val);
    else if(!strcmp(variable, "awb_gain")) res = s->set_awb_gain(s, val);
    else if(!strcmp(variable, "agc_gain")) res = s->set_agc_gain(s, val);
    else if(!strcmp(variable, "aec_value")) res = s->set_aec_value(s, val);
    else if(!strcmp(variable, "aec2")) res = s->set_aec2(s, val);
    else if(!strcmp(variable, "dcw")) res = s->set_dcw(s, val);
    else if(!strcmp(variable, "bpc")) res = s->set_bpc(s, val);
    else if(!strcmp(variable, "wpc")) res = s->set_wpc(s, val);
    else if(!strcmp(variable, "raw_gma")) res = s->set_raw_gma(s, val);
    else if(!strcmp(variable, "lenc")) res = s->set_lenc(s, val);
    else if(!strcmp(variable, "special_effect")) res = s->set_special_effect(s, val);
    else if(!strcmp(variable, "wb_mode")) res = s->set_wb_mode(s, val);
    else if(!strcmp(variable, "ae_level")) res = s->set_ae_level(s, val);
    else if(!strcmp(variable, "face_detect")) {
        detection_enabled = val;
        if(!detection_enabled) {
            recognition_enabled = 0;
        }
    }
    else if(!strcmp(variable, "face_enroll")) is_enrolling = val;
    else if(!strcmp(variable, "face_recognize")) {
        recognition_enabled = val;
        if(recognition_enabled){
            detection_enabled = val;
        }
    }
    else {
        res = -1;
    }

    if(res){
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t status_handler(httpd_req_t *req){
    static char json_response[1024];

    sensor_t * s = esp_camera_sensor_get();
    char * p = json_response;
    *p++ = '{';

    p+=sprintf(p, "\"framesize\":%u,", s->status.framesize);
    p+=sprintf(p, "\"quality\":%u,", s->status.quality);
    p+=sprintf(p, "\"brightness\":%d,", s->status.brightness);
    p+=sprintf(p, "\"contrast\":%d,", s->status.contrast);
    p+=sprintf(p, "\"saturation\":%d,", s->status.saturation);
    p+=sprintf(p, "\"sharpness\":%d,", s->status.sharpness);
    p+=sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
    p+=sprintf(p, "\"wb_mode\":%u,", s->status.wb_mode);
    p+=sprintf(p, "\"awb\":%u,", s->status.awb);
    p+=sprintf(p, "\"awb_gain\":%u,", s->status.awb_gain);
    p+=sprintf(p, "\"aec\":%u,", s->status.aec);
    p+=sprintf(p, "\"aec2\":%u,", s->status.aec2);
    p+=sprintf(p, "\"ae_level\":%d,", s->status.ae_level);
    p+=sprintf(p, "\"aec_value\":%u,", s->status.aec_value);
    p+=sprintf(p, "\"agc\":%u,", s->status.agc);
    p+=sprintf(p, "\"agc_gain\":%u,", s->status.agc_gain);
    p+=sprintf(p, "\"gainceiling\":%u,", s->status.gainceiling);
    p+=sprintf(p, "\"bpc\":%u,", s->status.bpc);
    p+=sprintf(p, "\"wpc\":%u,", s->status.wpc);
    p+=sprintf(p, "\"raw_gma\":%u,", s->status.raw_gma);
    p+=sprintf(p, "\"lenc\":%u,", s->status.lenc);
    p+=sprintf(p, "\"vflip\":%u,", s->status.vflip);
    p+=sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
    p+=sprintf(p, "\"dcw\":%u,", s->status.dcw);
    p+=sprintf(p, "\"colorbar\":%u,", s->status.colorbar);
    p+=sprintf(p, "\"face_detect\":%u,", detection_enabled);
    p+=sprintf(p, "\"face_enroll\":%u,", is_enrolling);
    p+=sprintf(p, "\"face_recognize\":%u", recognition_enabled);
    *p++ = '}';
    *p++ = 0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, strlen(json_response));
}

static esp_err_t index_handler(httpd_req_t *req){
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    sensor_t * s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        return httpd_resp_send(req, (const char *)index_ov3660_html_gz, index_ov3660_html_gz_len);
    }
    return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
}

//void startCameraServer(int NumFunction){
//    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
//
//
//    if(NumFunction == 5){
//      httpd_uri_t stream_uri = {
//        .uri       = "/stream",
//        .method    = HTTP_GET,
//        .handler   = stream_handler,
//        .user_ctx  = NULL
//      };
//
//      config.server_port += 1;
//      config.ctrl_port += 1;
//      Serial.printf("Starting stream server on port: '%d'\n", config.server_port);
//      if (httpd_start(&stream_httpd, &config) == ESP_OK) {
//          httpd_register_uri_handler(stream_httpd, &stream_uri);
//      }
//    }else{
//
//      httpd_uri_t index_uri = {
//          .uri       = "/",
//          .method    = HTTP_GET,
//          .handler   = index_handler,
//          .user_ctx  = NULL
//      };
//  
//      httpd_uri_t status_uri = {
//          .uri       = "/status",
//          .method    = HTTP_GET,
//          .handler   = status_handler,
//          .user_ctx  = NULL
//      };
//  
//      httpd_uri_t cmd_uri = {
//          .uri       = "/control",
//          .method    = HTTP_GET,
//          .handler   = cmd_handler,
//          .user_ctx  = NULL
//      };
//  
////      httpd_uri_t capture_uri = {
////          .uri       = "/capture",
////          .method    = HTTP_GET,
////          .handler   = capture_handler,
////          .user_ctx  = NULL
////      };  
//
//      Serial.printf("Starting web server on port: '%d'\n", config.server_port);
//      if (httpd_start(&camera_httpd, &config) == ESP_OK) {
//          httpd_register_uri_handler(camera_httpd, &index_uri);
//          httpd_register_uri_handler(camera_httpd, &cmd_uri);
//          httpd_register_uri_handler(camera_httpd, &status_uri);
//         // httpd_register_uri_handler(camera_httpd, &capture_uri);
//      }
//    } 
//
//
//    ra_filter_init(&ra_filter, 20);
//    
//    mtmn_config.type = FAST;
//    mtmn_config.min_face = 80;
//    mtmn_config.pyramid = 0.707;
//    mtmn_config.pyramid_times = 4;
//    mtmn_config.p_threshold.score = 0.6;
//    mtmn_config.p_threshold.nms = 0.7;
//    mtmn_config.p_threshold.candidate_number = 20;
//    mtmn_config.r_threshold.score = 0.7;
//    mtmn_config.r_threshold.nms = 0.7;
//    mtmn_config.r_threshold.candidate_number = 10;
//    mtmn_config.o_threshold.score = 0.7;
//    mtmn_config.o_threshold.nms = 0.7;
//    mtmn_config.o_threshold.candidate_number = 1;
//    
//    face_id_init(&id_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);    
//    
//}
