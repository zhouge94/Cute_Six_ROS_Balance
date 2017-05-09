#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pthread.h"
#include "/Robot/voice/inc/qisr.h"
#include "/Robot/voice/inc/msp_cmn.h"
#include "/Robot/voice/inc/msp_errors.h"
#include <stdio.h>
#include <stdlib.h>
#include <alsa/asoundlib.h>
#define SAMPLE_RATE 16000
#define CHANNLE 1
#define FRAMES_SIZE 3200
#define FORMAT SND_PCM_FORMAT_S16_LE
#define PER_SAMPLE 2
#define DEVICE	"default"
 char data[10000];
 int flag_begin=0;
 int flag_understand=0;
 int flag_unknow=0;

/* wav音频头部格式 */
typedef struct _wave_pcm_hdr
{
    char            riff[4];                // = "RIFF"
    int				size_8;                 // = FileSize - 8
    char            wave[4];                // = "WAVE"
    char            fmt[4];                 // = "fmt "
    int				fmt_size;				// = 下一个结构体的大小 : 16

    short int       format_tag;             // = PCM : 1
    short int       channels;               // = 通道数 : 1
    int				samples_per_sec;        // = 采样率 : 8000 | 6000 | 11025 | 16000
    int				avg_bytes_per_sec;      // = 每秒字节数 : samples_per_sec * bits_per_sample / 8
    short int       block_align;            // = 每采样点字节数 : wBitsPerSample / 8
    short int       bits_per_sample;        // = 量化比特数: 8 | 16

    char            data[4];                // = "data";
    int				data_size;              // = 纯数据长度 : FileSize - 44
} wave_pcm_hdr;
/* 默认wav音频头部数据 */
wave_pcm_hdr default_wav_hdr =
{
    { 'R', 'I', 'F', 'F' },
    0,
    {'W', 'A', 'V', 'E'},
    {'f', 'm', 't', ' '},
    16,
    1,
    1,
    16000,
    32000,
    2,
    16,
    {'d', 'a', 't', 'a'},
    0
};
#define SAMPLE_RATE_16K     (16000)
#define SAMPLE_RATE_8K      (8000)
#define MAX_GRAMMARID_LEN   (32)
#define MAX_PARAMS_LEN      (1024)


const char * ASR_RES_PATH        = "fo|/Robot/voice/bin/msc/res/asr/common.jet"; //离线语法识别资源路径
const char * GRM_BUILD_PATH      = "/Robot/voice/bin/msc/res/asr/GrmBuilld"; //构建离线语法识别网络生成数据保存路径
const char * GRM_FILE            = "/Robot/voice/call.bnf"; //构建离线识别语法网络所用的语法文件
const char * listened_file            ="/Robot/voice/wav/ddhgdw.pcm";
typedef struct _UserData {
	int     build_fini; //标识语法构建是否完成
	int     update_fini; //标识更新词典是否完成
	int     errcode; //记录语法构建或更新词典回调错误码
	char    grammar_id[MAX_GRAMMARID_LEN]; //保存语法构建返回的语法ID
}UserData;
UserData asr_data;
int build_grammar(UserData *udata); //构建离线识别语法网络
int run_asr(void *udata); //进行离线语法识别
int build_grm_cb(int ecode, const char *info, void *udata)
{
	UserData *grm_data = (UserData *)udata;
	if (NULL != grm_data) {
		grm_data->build_fini = 1;
		grm_data->errcode = ecode;
	}

	if (MSP_SUCCESS == ecode && NULL != info) {
		printf("构建语法成功！ 语法ID:%s\n", info);
		if (NULL != grm_data)
			snprintf(grm_data->grammar_id, MAX_GRAMMARID_LEN - 1, info);
	}
	else
		printf("构建语法失败！%d\n", ecode);

	return 0;
}

int build_grammar(UserData *udata)
{
	FILE *grm_file                           = NULL;
	char *grm_content                        = NULL;
	unsigned int grm_cnt_len                 = 0;
	char grm_build_params[MAX_PARAMS_LEN]    = {NULL};
	int ret                                  = 0;
	grm_file = fopen(GRM_FILE, "rb");	
	if(NULL == grm_file) {
		printf("打开\"%s\"文件失败！[%s]\n", GRM_FILE, strerror(errno));
		return -1; 
	}
	fseek(grm_file, 0, SEEK_END);
	grm_cnt_len = ftell(grm_file);
	fseek(grm_file, 0, SEEK_SET);

	grm_content = (char *)malloc(grm_cnt_len + 1);
	if (NULL == grm_content)
	{
		printf("内存分配失败!\n");
		fclose(grm_file);
		grm_file = NULL;
		return -1;
	}
	fread((void*)grm_content, 1, grm_cnt_len, grm_file);
	grm_content[grm_cnt_len] = '\0';
	fclose(grm_file);
	grm_file = NULL;

	snprintf(grm_build_params, MAX_PARAMS_LEN - 1, 
        "engine_type = local, \
		asr_res_path = %s, sample_rate = %d, \
		grm_build_path = %s, ",
		ASR_RES_PATH,
		SAMPLE_RATE_16K,
		GRM_BUILD_PATH
		);
	ret = QISRBuildGrammar("bnf", grm_content, grm_cnt_len, grm_build_params, build_grm_cb, udata);

	free(grm_content);
	grm_content = NULL;

	return ret;
}

int run_asr(void *ptr)
{
    char asr_params[MAX_PARAMS_LEN]    = {NULL};
    const char *rec_rslt               = NULL;
    const char *session_id             = NULL;
    const char *asr_audiof             = NULL;
    FILE *f_pcm                        = NULL;
    char *pcm_data                     = NULL;
    long pcm_count                     = 0;
    long pcm_size                      = 0;
    int last_audio                     = 0;
    int aud_stat                       = MSP_AUDIO_SAMPLE_CONTINUE;
    int ep_status                      = MSP_EP_LOOKING_FOR_SPEECH;
    int rec_status                     = MSP_REC_STATUS_INCOMPLETE;
    int rss_status                     = MSP_REC_STATUS_INCOMPLETE;
    int errcode                        = -1;
    asr_audiof =listened_file;
    f_pcm = fopen(asr_audiof, "rb");
    if (NULL == f_pcm) {
        printf("打开\"%s\"失败！[%s]\n", f_pcm, strerror(errno));
        goto run_error;
    }
    fseek(f_pcm, 0, SEEK_END);
    pcm_size = ftell(f_pcm);
    fseek(f_pcm, 0, SEEK_SET);
    pcm_data = (char *)malloc(pcm_size);
    if (NULL == pcm_data)goto run_error;
    fread((void *)pcm_data, pcm_size, 1, f_pcm);
    fclose(f_pcm);
    f_pcm = NULL;

    //离线语法识别参数设置
    snprintf(asr_params, MAX_PARAMS_LEN - 1,
        "engine_type = mixd, \
             sub=iat,\
             domain = iat,\
             sch=1,\
             nlp_version=2.0,\
             mixed_type=delay ,\
             mixed_threshold=10,\
             mixed_timeout= 10 ,\
        asr_res_path = %s, sample_rate = %d, \
        grm_build_path = %s, local_grammar = %s, \
        result_type = json, result_encoding = UTF-8, ",
        ASR_RES_PATH,
        SAMPLE_RATE_16K,
        GRM_BUILD_PATH,
        asr_data.grammar_id
        );
    session_id = QISRSessionBegin(NULL, asr_params, &errcode);
    if (NULL == session_id)	goto run_error;
    printf("开始识别...\n");

    while (1) {
        unsigned int len = 3200;

        if (pcm_size < 12800) {
            len = pcm_size;
            last_audio = 1;
        }

        aud_stat = MSP_AUDIO_SAMPLE_CONTINUE;

        if (0 == pcm_count)
            aud_stat = MSP_AUDIO_SAMPLE_FIRST;

        if (len <= 0)
            break;

        printf(">");
        fflush(stdout);
        errcode = QISRAudioWrite(session_id, (const void *)&pcm_data[pcm_count], len, aud_stat, &ep_status, &rec_status);
        if (MSP_SUCCESS != errcode)goto run_error;
        pcm_count += (long)len;
        pcm_size -= (long)len;
        if (MSP_EP_AFTER_SPEECH == ep_status)break;		//检测到音频结束
        usleep(100 * 1000); //模拟人说话时间间隙
    }
      printf("|");	//主动点击音频结束
    QISRAudioWrite(session_id, (const void *)NULL, 0, MSP_AUDIO_SAMPLE_LAST, &ep_status, &rec_status);

    free(pcm_data);
    pcm_data = NULL;

    //获取识别结果
    while (MSP_REC_STATUS_COMPLETE != rss_status && MSP_SUCCESS == errcode) {
        rec_rslt = QISRGetResult(session_id, &rss_status, 0, &errcode);
        printf(".");
        usleep(150 * 1000);
    }
    printf("\n识别结束：\n");
    printf("=============================================================\n");
    if (NULL != rec_rslt)
        printf("%s\n", rec_rslt);
    else
        printf("没有识别结果！\n");
    printf("=============================================================\n");

    goto run_exit;

run_error:
    if (NULL != pcm_data) {
        free(pcm_data);
        pcm_data = NULL;
    }
    if (NULL != f_pcm) {
        fclose(f_pcm);
        f_pcm = NULL;
    }
run_exit:
    printf("exit with code :%d..\n",errcode);
    QISRSessionEnd(session_id, NULL);
    return errcode;
}

int recode_asr(void *ptr)
{
    char asr_params[MAX_PARAMS_LEN]    = {NULL};
    const char *rec_rslt               = NULL;
    const char *session_id             = NULL;

    FILE *f_pcm                        = NULL;
    char *pcm_data                     = NULL;
    long pcm_count                     = 0;

    int aud_stat                       = MSP_AUDIO_SAMPLE_CONTINUE;
    int ep_status                      = MSP_EP_LOOKING_FOR_SPEECH;
    int rec_status                     = MSP_REC_STATUS_INCOMPLETE;
    int rss_status                     = MSP_REC_STATUS_INCOMPLETE;
    int errcode                        = -1;

    long loops;
    int rc, size;
    float time=5;
    snd_pcm_t *handle;
    snd_pcm_hw_params_t *params;
    snd_pcm_uframes_t frames, ret;
    char *buffer;
    char *ptr_buffer;
    FILE *fp = fopen("/Robot/voice/wav/listened.wav", "wb");

    //离线语法识别参数设置
    snprintf(asr_params, MAX_PARAMS_LEN - 1,
        "engine_type = cloud,\
             sub=iat,\
             domain = iat,\
             sch=1,\
             nlp_version=2.0,\
             mixed_type=delay ,\
             mixed_threshold=50,\
             mixed_timeout= 3000 ,\
        asr_res_path = %s, sample_rate = %d, \
        grm_build_path = %s, local_grammar = %s, \
        result_type = json, result_encoding = UTF-8 ",
        ASR_RES_PATH,
        SAMPLE_RATE_16K,
        GRM_BUILD_PATH,
        asr_data.grammar_id
        );
    session_id = QISRSessionBegin(NULL, asr_params, &errcode);
    printf("开始识别...\n");
    if(fp == NULL){  }else(printf("open file success"));
    rc = snd_pcm_open(&handle, DEVICE, SND_PCM_STREAM_CAPTURE, 0);
    if (rc < 0) {  } else  printf("OK:before alloca\n");
    snd_pcm_hw_params_alloca(&params);
    printf("OK:after alloca\n");
    rc = snd_pcm_hw_params_any(handle, params);
    if (rc < 0) {  }  else{    printf("OK:snd_pcm_hw_params_an()\n");  }
    rc = snd_pcm_hw_params_set_access(handle, params,SND_PCM_ACCESS_RW_INTERLEAVED);
    if (rc < 0) {  }  else{    printf("OK:snd_pcm_hw_params_set_access()\n");  }
    rc = snd_pcm_hw_params_set_format(handle, params, FORMAT);
    if (rc < 0) {  }  else{    printf("OK:snd_pcm_hw_params_set_format()\n");  }
    rc = snd_pcm_hw_params_set_channels(handle, params, CHANNLE);
    if (rc < 0) {  }  else{    printf("OK:snd_pcm_hw_params_set_channels()\n");  }
    rc = snd_pcm_hw_params_set_rate(handle, params,SAMPLE_RATE, 0);
    if (rc < 0) {  }  else{    printf("OK:snd_pcm_hw_params_set_rate()\n");  }
    frames = FRAMES_SIZE;
    rc = snd_pcm_hw_params(handle, params);
    if (rc < 0) {  }  else{    printf("OK:snd_pcm_hw_paraams()\n");  }
    size = frames * PER_SAMPLE *CHANNLE; /* 2 bytes/sample, 1 channels */
    ptr_buffer = buffer = (char *) malloc(size);
    if(buffer == NULL){  }else{    printf("OK:malloc()\n");  }
    loops = SAMPLE_RATE/frames*time;
    fwrite(&default_wav_hdr, sizeof(default_wav_hdr) ,1, fp);
    while (loops > 0)
    {
          loops--;
          ret = snd_pcm_readi(handle, ptr_buffer, frames);
          if (ret == -EPIPE) {
            printf( "overrun occurred\n");
            snd_pcm_prepare(handle);
          } else if (ret < 0)  printf("error from read: %s\n",snd_strerror(ret));
          else if (ret != frames)    printf( "short read, read %d frames\n", ret);
          rc = fwrite(ptr_buffer, size, 1, fp);
          if (rc < 0){   printf("error in write\n");   }
          if (rc != 1)    printf("failed to write %d bytes\n",size);
          printf(">");
          fflush(stdout);
        if (0 == pcm_count)	aud_stat = MSP_AUDIO_SAMPLE_FIRST;
        else   aud_stat = MSP_AUDIO_SAMPLE_CONTINUE;
        errcode = QISRAudioWrite(session_id, (const void *)ptr_buffer, size, aud_stat, &ep_status, &rec_status);
        if (MSP_EP_AFTER_SPEECH == ep_status)break;		//检测到音频结束
    }

    QISRAudioWrite(session_id, (const void *)NULL, 0, MSP_AUDIO_SAMPLE_LAST, &ep_status, &rec_status);

    if(buffer != NULL)  free(buffer);
    if(fp != NULL)  fclose(fp);
    snd_pcm_drain(handle);
    snd_pcm_close(handle);

    free(pcm_data);
    pcm_data = NULL;

    //获取识别结果
    while (MSP_REC_STATUS_COMPLETE != rss_status && MSP_SUCCESS == errcode) {
        rec_rslt = QISRGetResult(session_id, &rss_status, 0, &errcode);
        printf(".");
        usleep(150 * 1000);
    }
    printf("\n识别结束%d：\n",loops);
    printf("=============================================================\n");
    if (NULL != rec_rslt)
    {
        printf("%s\n", rec_rslt);
        sprintf(data,"%s\n", rec_rslt);
        flag_understand=1;
    }
    else
    {
        printf("没有识别结果！\n");
        flag_unknow=1;
    }
    printf("=============================================================\n");

    goto run_exit;

    if (NULL != pcm_data) {
        free(pcm_data);
        pcm_data = NULL;
    }
    if (NULL != f_pcm) {
        fclose(f_pcm);
        f_pcm = NULL;
    }
run_exit:
    printf("exit with code :%d..\n",errcode);
    QISRSessionEnd(session_id, NULL);
    return errcode;
}
void wakeupcallback(const std_msgs::String::ConstPtr& msg)
{
    std::cout<<"waking up"<<std::endl;
   // printf("%s", *msg->data.c_str());
    usleep(700*1000);
    flag_begin=1;
}

int main(int argc,char **argv)
{
    const char *login_config    = "appid = 573bdbff"; //登录参数
	int ret                     = 0 ;
    ret = MSPLogin(NULL, NULL, login_config); //第一个参数为用户名，第二个参数为密码，传NULL即可，第三个参数是登录参数
	if (MSP_SUCCESS != ret) {
		printf("登录失败：%d\n", ret);
        return 	-1;
	}
	printf("构建离线识别语法网络...\n");
	ret = build_grammar(&asr_data);  //第一次使用某语法进行识别，需要先构建语法网络，获取语法ID，之后使用此语法进行识别，无需再次构建
	if (MSP_SUCCESS != ret) {
		printf("构建语法调用失败！\n");
        return -1;
	}
    while (1 != asr_data.build_fini)	usleep(300 * 1000);
    if (MSP_SUCCESS != asr_data.errcode)	return -1;
    //printf("离线识别语法网络构建完成，开始识别...\n");
    if (MSP_SUCCESS != ret) {
        printf("离线语法识别出错: %d \n", ret);
       return -1;
    }
    ros::init(argc, argv, "xf_asr");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    ros::Subscriber sub = n.subscribe("xfwakeup", 1000, wakeupcallback);
    ros::Publisher pub = n.advertise<std_msgs::String>("xfsaywords", 1000);
    ros::Publisher pub2= n.advertise<std_msgs::String>("xfunderstand",10000);
    while (ros::ok())
    {
        if(flag_begin)
        {
            ret = recode_asr(&asr_data);
           // ret = run_asr(&asr_data);
         flag_begin=0;
        }
        if(flag_unknow)
        {
            std_msgs::String msg;
            std::stringstream ss;
            flag_unknow=0;
            ss << "对不起，我好像不明白！ ";
            msg.data = ss.str();
            pub.publish(msg);

        }
        if(flag_understand)
        {
            flag_understand=0;
            std_msgs::String msg;
            std::stringstream ss2;
            ss2 << data;
            msg.data = ss2.str();
            pub2.publish(msg);

        }
     ros::spinOnce();
     loop_rate.sleep();
    }

	MSPLogout();
	printf("请按任意键退出...\n");
	getchar();
}


