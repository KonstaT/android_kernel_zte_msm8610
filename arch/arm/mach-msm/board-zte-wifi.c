/* linux/arch/arm/mach-msm/board-zte-wifi.c
*/
#include <linux/platform_device.h>
#include <linux/if.h> /*For IFHWADDRLEN */
#include <linux/fs.h>
#include <linux/random.h>
#include <linux/jiffies.h>
#include <linux/export.h>

/* Function to get custom MAC address */
struct mac_addr
{
        char magic[6];
        int  valid;
        char addr[6];
};

int random_mac_addr(unsigned char *buf)
{
	unsigned int rand_mac;
	
    	srandom32((unsigned int)jiffies);
    	rand_mac = random32();
    	buf[0] = 0x00;
    	buf[1] = 0xd0;
    	buf[2] = 0xd0;
	buf[3] = (unsigned char)rand_mac;
	buf[4] = (unsigned char)(rand_mac >> 8);
	buf[5] = (unsigned char)(rand_mac >> 16);	

	printk("wifi mac: %02x:%02x:%02x:%02x:%02x:%02x\n", 
		buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

	return 0;
}

int
zte_wifi_get_mac_addr(unsigned char *addr)
{
#if 0
	int rc = 0;
	#define WIFI_MAC_ADDR_FILE    "/persist/wifimac.dat"
	#define WIFI_MAC_MAGIC        "wfaddr"

	#define MAC_MAGIC_LEN      6

	struct file *fp;
	struct mac_addr mWifiMac;
	char * buf = (char *)&mWifiMac;
	memset(buf,0,sizeof(mWifiMac));
	fp = filp_open(WIFI_MAC_ADDR_FILE, O_RDONLY, 0);

	 if (IS_ERR(fp))
	 {
		 fp = NULL;
		 printk("Open wifi mac file failed! skip it and use random mac addr\n");
		 random_mac_addr(addr);
		 return -1;
	 }
	 
	// printk("Mac addr = %.2x:%.2x:%.2x:%.2x:%.2x:%.2x",mWifiMac.addr[0],mWifiMac.addr[1],mWifiMac.addr[2],mWifiMac.addr[3],mWifiMac.addr[4],mWifiMac.addr[5]);

	rc = kernel_read(fp, fp->f_pos, buf, sizeof(mWifiMac));
	printk("=====================================\n");
	printk("=======Wifi Mac Data from file=======\n");
	printk("=====================================\n");
	printk("rc=%d , magic=%s ,valid=%d\n",rc,mWifiMac.magic,mWifiMac.valid);
	printk("Mac addr = %.2x:%.2x:%.2x:%.2x:%.2x:%.2x\n",mWifiMac.addr[0],mWifiMac.addr[1],mWifiMac.addr[2],mWifiMac.addr[3],mWifiMac.addr[4],mWifiMac.addr[5]);
	
	if(rc&&!strcmp(mWifiMac.magic,WIFI_MAC_MAGIC)&&mWifiMac.valid) {
		memcpy(addr,mWifiMac.addr, IFHWADDRLEN);
		printk("get wifi mac successfully! \n");			
	} else {
 		printk("get wifi mac fail, use random mac addr \n");	
		random_mac_addr(addr);
		return -1;
	}
	
	filp_close(fp, NULL);
	return 0;
#endif
	
	#define WIFI_MAC_ADDR_FILE    "/persist/wifimac.dat"
	#define WIFI_MAX_ADDR_LEN     60
	#define WIFI_MAC_ADDR_HEAD    "wifiaddr:"
	int rc = 0;
	char buf[WIFI_MAX_ADDR_LEN];
	char *s;
	struct file *fp;
	unsigned int wifi_addr[6];
	int i;
	
	memset(buf, 0, WIFI_MAX_ADDR_LEN);
	fp = filp_open(WIFI_MAC_ADDR_FILE, O_RDONLY, 0);

	if (IS_ERR(fp)){
		fp = NULL;
		printk("Open wifi mac file failed! skip it and use random mac addr\n");
		random_mac_addr(addr);
		return -1;
	}
	 
	rc = kernel_read(fp, fp->f_pos, buf, WIFI_MAX_ADDR_LEN);
	s = strstr(buf, WIFI_MAC_ADDR_HEAD);
	printk("Read form mac file - %s", s);	
	if (!s) {
		printk("get wifi mac fail, use random mac addr \n");
		random_mac_addr(addr);
		filp_close(fp, NULL);
		return -1;
	}
	else{
		sscanf(s, "wifiaddr:0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", 
			&wifi_addr[0], &wifi_addr[1], &wifi_addr[2], 
			&wifi_addr[3], &wifi_addr[4], &wifi_addr[5]);
		for(i=0; i<6; i++)
			addr[i] = wifi_addr[i];
		if(0 != ((unsigned int)addr[0] % 2)){
			printk("get wifi mac fail, use random mac addr \n");
			random_mac_addr(addr);
			filp_close(fp, NULL);
			return -1;
		}
		else{
			printk("get wifi mac successfully! \n");
			filp_close(fp, NULL);
			return 0;
		}
	}
}

EXPORT_SYMBOL(zte_wifi_get_mac_addr);

