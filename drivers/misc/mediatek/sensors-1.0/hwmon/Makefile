ifeq ($(CONFIG_MTK_SENSOR_SUPPORT),y)
obj-y += hwmsen/
obj-y += sensor_dts/
obj-y += sensor_attributes/
obj-y += sensor_event/
obj-y += sensor_performance/
#ifdef ODM_HQ_EDIT */
#/* zuoqiquan@ODM_HQ.Sensors.SCP.BSP, 2018/11/26,for sensors cts/vts fail  */
obj-y += sensor_list/
#endif /* ODM_HQ_EDIT */
else
obj-n := hwmsen/
endif
