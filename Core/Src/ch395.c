/**
 ****************************************************************************************************
 * @file        ch395q.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-6-17
 * @brief       CH395 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F103开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "ch395.h"
#include "spi.h"
#include "main.h"
#include "iwdg.h"

struct ch395q_t g_ch395q_sta;

extern uint8_t work_mode;

/**
 * @brief       ch395_gpio初始化
 * @param       无
 * @retval      无
 */
void ch395_gpio_init( void )
{
    HAL_GPIO_WritePin(CH395_RST_GPIO_Port, CH395_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
}

/**
 * @brief       硬件SPI输出且输入8个位数据
 * @param       d:将要送入到ch395的数据
 * @retval      无
 */
uint8_t ch395_read_write_byte( uint8_t data )
{
    uint8_t rxdata;
    rxdata = spi1_read_write_byte(data);       /* SPI写入一个CH395Q数据并返回一个数据 */
    return rxdata;                             /* 返回收到的数据 */
}

/**
 * @brief       向ch395写命令
 * @param       将要写入ch395的命令码
 * @retval      无
 */
void ch395_write_cmd( uint8_t mcmd )
{
    ch395_scs_hign;                         /* 防止CS原来为低，先将CS置高 */
    ch395_scs_low;                          /* 命令开始，CS拉低 */
    ch395_read_write_byte(mcmd);            /* SPI发送命令码 */
    HAL_Delay(2);                            /* 必要延时,延时1.5uS确保读写周期不小于1.5uS */

}

/**
 * @brief       向ch395写数据
 * @param       将要写入ch395的数据
 * @retval      无
 */
void ch395_write_data( uint8_t mdata )
{
    ch395_read_write_byte(mdata);           /* SPI发送数据 */
}

/**
 * @brief       从ch395读数据
 * @param       无
 * @retval      返回读取的数据
 */
uint8_t ch395_read_data( void )
{
    uint8_t i;
    i = ch395_read_write_byte(0xff);        /* SPI读数据 */
    return i;
}

/**
 * @brief       ch395_keeplive_set 保活定时器参数设置
 * @param       无
 * @retval      无
 */
void ch395_keeplive_set(void)
{
    ch395_keeplive_cnt(DEF_KEEP_LIVE_CNT);
    ch395_keeplive_idle(DEF_KEEP_LIVE_IDLE);
    ch395_keeplive_intvl(DEF_KEEP_LIVE_PERIOD);
}

/**
 * @brief       ch395 socket配置
 * @param       ch395_sokect：Socket配置信息
 * @retval      无
 */
uint8_t ch395q_socket_config(ch395_socket * ch395_sokect)
{
    if (ch395_sokect == NULL)
    {
        return 0;
    }

    if (g_ch395q_sta.dhcp_status == DHCP_UP)                                    /* DHCP获取成功状态 */
    {
        ch395_sokect->net_info.ip[0] = g_ch395q_sta.ipinf_buf[0];
        ch395_sokect->net_info.ip[1] = g_ch395q_sta.ipinf_buf[1];
        ch395_sokect->net_info.ip[2] = g_ch395q_sta.ipinf_buf[2];
        ch395_sokect->net_info.ip[3] = g_ch395q_sta.ipinf_buf[3];

        ch395_sokect->net_info.gwip[0] = g_ch395q_sta.ipinf_buf[4];
        ch395_sokect->net_info.gwip[1] = g_ch395q_sta.ipinf_buf[5];
        ch395_sokect->net_info.gwip[2] = g_ch395q_sta.ipinf_buf[6];
        ch395_sokect->net_info.gwip[3] = g_ch395q_sta.ipinf_buf[7];

        ch395_sokect->net_info.mask[0] = g_ch395q_sta.ipinf_buf[8];
        ch395_sokect->net_info.mask[1] = g_ch395q_sta.ipinf_buf[9];
        ch395_sokect->net_info.mask[2] = g_ch395q_sta.ipinf_buf[10];
        ch395_sokect->net_info.mask[3] = g_ch395q_sta.ipinf_buf[11];

        ch395_sokect->net_info.dns1[0] = g_ch395q_sta.ipinf_buf[12];
        ch395_sokect->net_info.dns1[1] = g_ch395q_sta.ipinf_buf[13];
        ch395_sokect->net_info.dns1[2] = g_ch395q_sta.ipinf_buf[14];
        ch395_sokect->net_info.dns1[3] = g_ch395q_sta.ipinf_buf[15];

        ch395_sokect->net_info.dns2[0] = g_ch395q_sta.ipinf_buf[16];
        ch395_sokect->net_info.dns2[1] = g_ch395q_sta.ipinf_buf[17];
        ch395_sokect->net_info.dns2[2] = g_ch395q_sta.ipinf_buf[18];
        ch395_sokect->net_info.dns2[3] = g_ch395q_sta.ipinf_buf[19];
    }
    else                                                                      /* DHCP获取失败状态，设置静态IP地址信息 */
    {
        ch395_cmd_set_ipaddr(ch395_sokect->net_config.ipaddr);                /* 设置CH395的IP地址 */
        ch395_cmd_set_gw_ipaddr(ch395_sokect->net_config.gwipaddr);           /* 设置网关地址 */
        ch395_cmd_set_maskaddr(ch395_sokect->net_config.maskaddr);            /* 设置子网掩码，默认为255.255.255.0*/
        ch395_cmd_init();
        HAL_Delay(10);
    }

    ch395_cmd_set_macaddr(ch395_sokect->net_config.macaddr);                  /* 设置MAC地址 */

    memcpy(&g_ch395q_sta.socket[ch395_sokect->socket_index].config, ch395_sokect, sizeof(ch395_socket));

    switch(ch395_sokect->proto)
    {
        case CH395Q_SOCKET_UDP:
            /* socket 为UDP模式 */
            ch395_set_socket_desip(ch395_sokect->socket_index, ch395_sokect->des_ip);                                           /* 设置socket 0目标IP地址 */
            ch395_set_socket_prot_type(ch395_sokect->socket_index,  PROTO_TYPE_UDP);                                            /* 设置socket 0协议类型 */
            ch395_set_socket_desport(ch395_sokect->socket_index, ch395_sokect->des_port);                                       /* 设置socket 0目的端口 */
            ch395_set_socket_sourport(ch395_sokect->socket_index, ch395_sokect->sour_port);                                     /* 设置socket 0源端口 */
            g_ch395q_sta.ch395_error(ch395_open_socket(ch395_sokect->socket_index));                                            /* 检查是否成功 */
            break;
        case CH395Q_SOCKET_TCP_CLIENT:
            /* socket 为TCPClient模式 */
            ch395_keeplive_set();                                                                                               /* 保活设置 */
            ch395_set_socket_desip(ch395_sokect->socket_index, ch395_sokect->des_ip);                                           /* 设置socket 0目标IP地址 */
            ch395_set_socket_prot_type(ch395_sokect->socket_index,  PROTO_TYPE_TCP);                                            /* 设置socket 0协议类型 */
            ch395_set_socket_desport(ch395_sokect->socket_index, ch395_sokect->des_port);                                       /* 设置socket 0目的端口 */
            ch395_set_socket_sourport(ch395_sokect->socket_index, ch395_sokect->sour_port);                                     /* 设置socket 0源端口 */
            g_ch395q_sta.ch395_error(ch395_open_socket(ch395_sokect->socket_index));                                            /* 检查sokect是否打开成功 */
            g_ch395q_sta.ch395_error(ch395_tcp_connect(ch395_sokect->socket_index));                                            /* 检查tcp连接是否成功 */
            break;
        case CH395Q_SOCKET_TCP_SERVER:
            /* socket 为TCPServer模式 */
            ch395_set_socket_desip(ch395_sokect->socket_index, ch395_sokect->des_ip);                                           /* 设置socket 0目标IP地址 */
            ch395_set_socket_prot_type(ch395_sokect->socket_index,  PROTO_TYPE_TCP);                                            /* 设置socket 0协议类型 */
            ch395_set_socket_sourport(ch395_sokect->socket_index, ch395_sokect->sour_port);                                     /* 设置socket 0源端口 */
            g_ch395q_sta.ch395_error(ch395_open_socket(ch395_sokect->socket_index));                                            /* 检查sokect是否打开成功 */
            g_ch395q_sta.ch395_error(ch395_tcp_listen(ch395_sokect->socket_index));                                             /* 监听tcp连接 */
            break;
        case CH395Q_SOCKET_MAC_RAW:
            ch395_set_socket_prot_type(ch395_sokect->socket_index,  PROTO_TYPE_MAC_RAW);                                        /* 设置socket 0协议类型 */
            g_ch395q_sta.ch395_error(ch395_open_socket(ch395_sokect->socket_index));                                            /* 检查sokect是否打开成功 */
            break;
    }

    return 1;
}

/**
 * @brief       调试使用，显示错误代码，并停机
 * @param       ierror 检测命令
 * @retval      无
 */
void ch395_error(uint8_t ierror)
{
    if (ierror == CMD_ERR_SUCCESS)
    {
        return;          /* 操作成功 */
    }

    printf("Error: %02X\r\n", (uint16_t)ierror);    /* 显示错误 */

    while ( 1 )
    {
        HAL_Delay(200);
        HAL_Delay(200);
    }
}

/**
 * @brief       CH395 PHY状态
 * @param       phy_status：PHY状态值
 * @retval      无
 */
void ch395_phy_status(uint8_t phy_status)
{
    switch (phy_status)
    {
        case PHY_DISCONN:
            printf("PHY DISCONN\r\n");
            break;
        case PHY_10M_FLL:
            printf("PHY 10M_FLL\r\n");
            break;
        case PHY_10M_HALF:
            printf("PHY 10M_HALF\r\n");
            break;
        case PHY_100M_FLL:
            printf("PHY 100M_FLL\r\n");
            break;
        case PHY_100M_HALF:
            printf("PHY 100M_HALF\r\n");
            break;
        default:
            printf("PHY AUTO\r\n");
            break;
    }

    /* 喂狗 */
    HAL_IWDG_Refresh(&hiwdg);
    HAL_Delay(1000);
}

/**
 * @brief      设置socket接口的接收与发送缓冲区
 * @param      无
 * @retval     无
 */
void ch395_socket_r_s_buf_modify(void)
{

   ch395_set_socket_recv_buf(0,0,4);                                                            /* Socket 0 ，接收缓冲区4*512 = 2K，发送缓冲区2*512 = 1K*/
   ch395_set_socket_send_buf(0,4,2);

   ch395_set_socket_recv_buf(1,6,4);                                                            /* Socket 1 */
   ch395_set_socket_send_buf(1,10,2);

   ch395_set_socket_recv_buf(2,12,4);                                                           /* Socket 2 */
   ch395_set_socket_send_buf(2,16,2);

   ch395_set_socket_recv_buf(3,18,4);                                                           /* Socket 3 */
   ch395_set_socket_send_buf(3,22,2);

   ch395_set_socket_recv_buf(4,24,4);                                                           /* Socket 4 */
   ch395_set_socket_send_buf(4,28,2);

   ch395_set_socket_recv_buf(5,30,4);                                                           /* Socket 5 */
   ch395_set_socket_send_buf(5,34,2);

   ch395_set_socket_recv_buf(6,36,4);                                                           /* Socket 6 */
   ch395_set_socket_send_buf(6,40,2);

   ch395_set_socket_recv_buf(7,42,4);                                                           /* Socket 7 */
   ch395_set_socket_send_buf(7,46,2);

}

/**
 * @brief      ch395_tcp初始化
 * @param      无
 * @retval     无
 */
void ch395_hardware_init(void)
{
    uint8_t i;
    ch395_gpio_init();

    g_ch395q_sta.ch395_error = ch395_error;
    g_ch395q_sta.ch395_phy_cb = ch395_phy_status;
    g_ch395q_sta.ch395_reconnection = ch395_reconnection;
    g_ch395q_sta.dhcp_status = DHCP_STA;

    i = ch395_cmd_check_exist(0x65);                                        /* 测试命令，用于测试硬件以及接口通讯 */

    if (i != 0x9a)
    {
        g_ch395q_sta.ch395_error(i);                                        /* ch395q检测错误 */
    }

    ch395_cmd_reset();                                                      /* 对ch395q复位 */
    HAL_Delay(100);                                                          /* 这里必须等待100以上延时 */

    g_ch395q_sta.ch395_error(ch395_cmd_init());                             /* 初始化ch395q命令 */
    ch395_socket_r_s_buf_modify();
//      ch395_set_tcpmss(536);
//      ch395_set_start_para(FUN_PARA_FLAG_TCP_SERVER | SOCK_CTRL_FLAG_SOCKET_CLOSE);

    do
    {
        g_ch395q_sta.phy_status = ch395_cmd_get_phy_status();               /* 获取PHY状态 */
        g_ch395q_sta.ch395_phy_cb(g_ch395q_sta.phy_status);                 /* 判断双工和网速模式 */
    }
    while(g_ch395q_sta.phy_status == PHY_DISCONN);

    g_ch395q_sta.version = ch395_cmd_get_ver();                             /* 获取版本 */
    printf("CH395VER : %2x\r\n", (uint16_t)g_ch395q_sta.version);

    i = ch395_dhcp_enable(1);                                               /* 开启DHCP */
    g_ch395q_sta.ch395_error(i);                                            /* ch395q检测错误 */

    HAL_Delay(1000);                                                         /* ch395q初始化延时 */
}

/**
 * @brief       CH395 socket 中断,在全局中断中被调用
 * @param       sockindex （0~7）
 * @retval      无
 */
void ch395_socket_interrupt(uint8_t sockindex)
{
    uint8_t  sock_int_socket;
    uint16_t rx_len = 0;

    sock_int_socket = ch395_get_socket_int(sockindex);      /* 获取socket 的中断状态 */

    if (sock_int_socket & SINT_STAT_SENBUF_FREE)            /* 发送缓冲区空闲，可以继续写入要发送的数据 */
    {

    }

    if (sock_int_socket & SINT_STAT_SEND_OK)                /* 发送完成中断 */
    {
    	/* 若发送完成，则喂狗，否则说明发送失败，连续发送失败则说明已与服务端断开，则看门狗复位重启传感器 */
    	HAL_IWDG_Refresh(&hiwdg);
    }

    if (sock_int_socket & SINT_STAT_RECV)                   /* 接收中断 */
    {
        g_ch395q_sta.socket[sockindex].config.recv.size = ch395_get_recv_length(sockindex);     /* 获取当前缓冲区内数据长度 */
        rx_len = g_ch395q_sta.socket[sockindex].config.recv.size;
        ch395_get_recv_data(sockindex, rx_len, g_ch395q_sta.socket[sockindex].config.recv.buf); /* 读取数据 */
        g_ch395q_sta.socket[sockindex].config.recv.buf[rx_len] = '\0';
//        printf("%s", g_ch395q_sta.socket[sockindex].config.recv.buf);
        g_ch395q_sta.socket[sockindex].config.recv.recv_flag |= 0x04;
    }

    if (sock_int_socket & SINT_STAT_CONNECT)                /* 连接中断，仅在TCP模式下有效 */
    {
        if (g_ch395q_sta.socket[sockindex].config.proto == CH395Q_SOCKET_TCP_CLIENT)
        {
            ch395_set_keeplive(sockindex,1);                /* 打开KEEPALIVE保活定时器 */
            ch395_setttl_num(sockindex,60);                 /* 设置TTL */
        }
    }

    if (sock_int_socket & SINT_STAT_DISCONNECT)             /* 断开中断，仅在TCP模式下有效 */
    {
        g_ch395q_sta.ch395_error(ch395_open_socket(g_ch395q_sta.socket[sockindex].config.socket_index));

        switch(g_ch395q_sta.socket[sockindex].config.proto)
        {
            case CH395Q_SOCKET_TCP_CLIENT:
                g_ch395q_sta.ch395_error(ch395_tcp_connect(g_ch395q_sta.socket[sockindex].config.socket_index));
                break;
            case CH395Q_SOCKET_TCP_SERVER:
                g_ch395q_sta.ch395_error(ch395_tcp_listen(g_ch395q_sta.socket[sockindex].config.socket_index));
                break;
            default:
                break;
        }
        HAL_Delay(200);                                      /* 延时200MS后再次重试，没有必要过于频繁连接 */
    }

    if (sock_int_socket & SINT_STAT_TIM_OUT)                /* 超时中断，仅在TCP模式下有效 */
    {
        if (g_ch395q_sta.socket[sockindex].config.proto == CH395Q_SOCKET_TCP_CLIENT)
        {
            HAL_Delay(200);                                  /* 延时200MS后再次重试，没有必要过于频繁连接 */
            g_ch395q_sta.ch395_error(ch395_open_socket(g_ch395q_sta.socket[sockindex].config.socket_index));
            g_ch395q_sta.ch395_error(ch395_tcp_connect(g_ch395q_sta.socket[sockindex].config.socket_index));
        }
    }
}

/**
 * @brief       CH395全局中断函数
 * @param       无
 * @retval      无
 */
void ch395_interrupt_handler(void)
{
    uint16_t  init_status;
    uint8_t i;

    init_status = ch395_cmd_get_glob_int_status_all();

    if (init_status & GINT_STAT_UNREACH)                                    /* 不可达中断，读取不可达信息 */
    {
         ch395_cmd_get_unreachippt(g_ch395q_sta.ipinf_buf);
    }

    if (init_status & GINT_STAT_IP_CONFLI)                                  /* 产生IP冲突中断，建议重新修改CH395的 IP，并初始化CH395 */
    {

    }

    if (init_status & GINT_STAT_PHY_CHANGE)                                 /* 产生PHY改变中断 */
    {
        g_ch395q_sta.phy_status = ch395_cmd_get_phy_status();               /* 获取PHY状态 */
    }

    if (init_status & GINT_STAT_DHCP)                                       /* 处理DHCP中断 */
    {

        i = ch395_get_dhcp_status();
        if(work_mode == 0) i = DHCP_DOWN;

        switch (i)
        {
            case DHCP_UP:
                ch395_get_ipinf(g_ch395q_sta.ipinf_buf);
                printf("IP:%02d.%02d.%02d.%02d\r\n", (uint16_t)g_ch395q_sta.ipinf_buf[0], (uint16_t)g_ch395q_sta.ipinf_buf[1], (uint16_t)g_ch395q_sta.ipinf_buf[2], (uint16_t)g_ch395q_sta.ipinf_buf[3]);
                printf("GWIP:%02d.%02d.%02d.%02d\r\n", (uint16_t)g_ch395q_sta.ipinf_buf[4], (uint16_t)g_ch395q_sta.ipinf_buf[5], (uint16_t)g_ch395q_sta.ipinf_buf[6], (uint16_t)g_ch395q_sta.ipinf_buf[7]);
                printf("Mask:%02d.%02d.%02d.%02d\r\n", (uint16_t)g_ch395q_sta.ipinf_buf[8], (uint16_t)g_ch395q_sta.ipinf_buf[9], (uint16_t)g_ch395q_sta.ipinf_buf[10], (uint16_t)g_ch395q_sta.ipinf_buf[11]);
                printf("DNS1:%02d.%02d.%02d.%02d\r\n", (uint16_t)g_ch395q_sta.ipinf_buf[12], (uint16_t)g_ch395q_sta.ipinf_buf[13], (uint16_t)g_ch395q_sta.ipinf_buf[14], (uint16_t)g_ch395q_sta.ipinf_buf[15]);
                printf("DNS2:%02d.%02d.%02d.%02d\r\n", (uint16_t)g_ch395q_sta.ipinf_buf[16], (uint16_t)g_ch395q_sta.ipinf_buf[17], (uint16_t)g_ch395q_sta.ipinf_buf[18], (uint16_t)g_ch395q_sta.ipinf_buf[19]);
                g_ch395q_sta.dhcp_status = DHCP_UP;
                break;
            default:
                g_ch395q_sta.dhcp_status = DHCP_DOWN;
                /* 设置默认IP地址信息 */
                printf("Static IP Mode.....................................\r\n");
                break;
        }
    }

    if (init_status & GINT_STAT_SOCK0)
    {
        ch395_socket_interrupt(CH395Q_SOCKET_0);                          /* 处理socket 0中断 */
    }

    if (init_status & GINT_STAT_SOCK1)
    {
        ch395_socket_interrupt(CH395Q_SOCKET_1);                          /* 处理socket 1中断 */
    }

    if (init_status & GINT_STAT_SOCK2)
    {
        ch395_socket_interrupt(CH395Q_SOCKET_2);                          /* 处理socket 2中断 */
    }

    if (init_status & GINT_STAT_SOCK3)
    {
        ch395_socket_interrupt(CH395Q_SOCKET_3);                          /* 处理socket 3中断 */
    }

    if (init_status & GINT_STAT_SOCK4)
    {
        ch395_socket_interrupt(CH395Q_SOCKET_4);                          /* 处理socket 4中断 */
    }

    if (init_status & GINT_STAT_SOCK5)
    {
        ch395_socket_interrupt(CH395Q_SOCKET_5);                          /* 处理socket 5中断 */
    }

    if (init_status & GINT_STAT_SOCK6)
    {
        ch395_socket_interrupt(CH395Q_SOCKET_6);                          /* 处理socket 6中断 */
    }

    if (init_status & GINT_STAT_SOCK7)
    {
        ch395_socket_interrupt(CH395Q_SOCKET_7);                          /* 处理socket 7中断 */
    }
}

/**
 * @brief       CH395全局管理函数
 * @param       无
 * @retval      无
 */
void ch395q_handler(void)
{
    if (ch395_int_pin_wire == 0)
    {
        ch395_interrupt_handler();                                       /* 中断处理函数 */
    }

    g_ch395q_sta.ch395_reconnection();                                   /* 检测PHY状态，并重新连接 */
}

/**
 * @brief       检测PHY状态，并重新连接
 * @param       无
 * @retval      无
 */
void ch395_reconnection(void)
{
    for (uint8_t socket_index = CH395Q_SOCKET_0 ; socket_index <= CH395Q_SOCKET_7 ; socket_index ++ )
    {
        if (g_ch395q_sta.phy_status == PHY_DISCONN && (g_ch395q_sta.dhcp_status == DHCP_UP || g_ch395q_sta.dhcp_status == DHCP_DOWN || g_ch395q_sta.dhcp_status == DHCP_STA))
        {
            if (g_ch395q_sta.socket[socket_index].config.socket_enable == CH395Q_ENABLE)
            {
                ch395_close_socket(g_ch395q_sta.socket[socket_index].config.socket_index);
                g_ch395q_sta.ch395_error(ch395_dhcp_enable(0));                                                                 /* ch395q检测错误 */
                g_ch395q_sta.socket[socket_index].config.socket_enable = CH395Q_DISABLE;
                g_ch395q_sta.dhcp_status = DHCP_STA;
            }

            printf("PHY DISCONN\r\n");
        }
        else
        {
            if (g_ch395q_sta.phy_status != PHY_DISCONN && g_ch395q_sta.socket[socket_index].config.socket_enable == CH395Q_DISABLE)
            {
                if (g_ch395q_sta.dhcp_status == DHCP_STA)
                {
                    ch395_cmd_reset();                                                                                          /* 对ch395q复位 */
                    HAL_Delay(100);                                                                                              /* 这里必须等待100以上延时 */
                    ch395_cmd_init();
                    HAL_Delay(100);                                                                                              /* 这里必须等待100以上延时 */
                    ch395_socket_r_s_buf_modify();
//                    ch395_set_tcpmss(536);
//                    ch395_set_start_para(FUN_PARA_FLAG_TCP_SERVER | SOCK_CTRL_FLAG_SOCKET_CLOSE);
                    g_ch395q_sta.ch395_error(ch395_dhcp_enable(1));                                                             /* 开启DHCP */
                }

                do
                {
                    if (ch395_int_pin_wire == 0)
                    {
                        ch395_interrupt_handler();                                                                              /* 中断处理函数 */
                    }
                }
                while (g_ch395q_sta.dhcp_status == DHCP_STA);                                                                   /* 获取DHCP */

                switch(g_ch395q_sta.socket[socket_index].config.proto)
                {
                    case CH395Q_SOCKET_UDP:
                        /* socket 为UDP模式 */
                        ch395_set_socket_desip(socket_index, g_ch395q_sta.socket[socket_index].config.des_ip);                  /* 设置socket 0目标IP地址 */
                        ch395_set_socket_prot_type(socket_index,  PROTO_TYPE_UDP);                                              /* 设置socket 0协议类型 */
                        ch395_set_socket_desport(socket_index, g_ch395q_sta.socket[socket_index].config.des_port);              /* 设置socket 0目的端口 */
                        ch395_set_socket_sourport(socket_index, g_ch395q_sta.socket[socket_index].config.sour_port);            /* 设置socket 0源端口 */
                        g_ch395q_sta.ch395_error(ch395_open_socket(socket_index));                                              /* 检查是否成功 */
                        break;
                    case CH395Q_SOCKET_TCP_CLIENT:
                        /* socket 为TCPClient模式 */
                        ch395_keeplive_set();                                                                                   /* 保活设置 */
                        ch395_set_socket_desip(socket_index, g_ch395q_sta.socket[socket_index].config.des_ip);                  /* 设置socket 0目标IP地址 */
                        ch395_set_socket_prot_type(socket_index,  PROTO_TYPE_TCP);                                              /* 设置socket 0协议类型 */
                        ch395_set_socket_desport(socket_index,g_ch395q_sta.socket[socket_index].config.des_port);               /* 设置socket 0目的端口 */
                        ch395_set_socket_sourport(socket_index,g_ch395q_sta.socket[socket_index].config.sour_port);             /* 设置socket 0源端口 */
                        g_ch395q_sta.ch395_error(ch395_open_socket(socket_index));                                              /* 检查sokect是否打开成功 */
                        g_ch395q_sta.ch395_error(ch395_tcp_connect(socket_index));                                              /* 检查tcp连接是否成功 */
                        break;
                    case CH395Q_SOCKET_TCP_SERVER:
                        /* socket 为TCPServer模式 */
                        ch395_set_socket_desip(socket_index, g_ch395q_sta.socket[socket_index].config.des_ip);                  /* 设置socket 0目标IP地址 */
                        ch395_set_socket_prot_type(socket_index,  PROTO_TYPE_TCP);                                              /* 设置socket 0协议类型 */
                        ch395_set_socket_sourport(socket_index, g_ch395q_sta.socket[socket_index].config.sour_port);            /* 设置socket 0源端口 */
                        g_ch395q_sta.ch395_error(ch395_open_socket(socket_index));                                              /* 检查sokect是否打开成功 */
                        g_ch395q_sta.ch395_error(ch395_tcp_listen(socket_index));                                               /* 监听tcp连接 */
                        break;
                    case CH395Q_SOCKET_MAC_RAW:
                        ch395_set_socket_prot_type(socket_index,  PROTO_TYPE_MAC_RAW);                                          /* 设置socket 0协议类型 */
                        g_ch395q_sta.ch395_error(ch395_open_socket(socket_index));                                              /* 检查sokect是否打开成功 */
                        break;
                    default:
                        ch395_set_socket_prot_type(socket_index,  PROTO_TYPE_TCP);
                        ch395_set_socket_sourport(socket_index, 8080);                                                          /* 设置socket 1~7源端口 */
                        break;
                }
                g_ch395q_sta.socket[socket_index].config.socket_enable = CH395Q_ENABLE;
            }
        }
    }
}
