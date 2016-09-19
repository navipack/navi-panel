
#include "adc_user.h"
#include "contact_detection.h"

typedef struct contactTypeStruct
{
    double contact_r;
    double contact_l;
    double R_r;
    double R_l;
    u8 currentChannel;
}contactType;

contactType  contactData;

void Contact_detection_V(void)
{
    ADC_GetRef();
    int contact1,contact2;
    int i;
//    double a[10];
    for(i=0,contact1=0,contact2=0;i<32;i++)
    {
        contact1=contact1+ADC_GetValue(6);
        contact2=contact2+ADC_GetValue(7);
        HAL_Delay(1);
    }
//    for(i=0;i<10;i++)
//    {
//        a[i]=(s32)ADC_GetValue(i);
//    }

    contactData.contact_l= contact1>>5;
    contactData.contact_r= contact2>>5;
    contactData.R_l=(4096000/contactData.contact_l)-1000;    //分压电阻为200欧姆
    contactData.R_r=(4096000/contactData.contact_r)-1000;
//    contactData.R_l=contactData.contact_l*3.3/4096;    //分压电阻为200欧姆
//    contactData.R_r=contactData.contact_r*3.3/4096;
}
