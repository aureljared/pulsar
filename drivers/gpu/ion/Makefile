ccflags-y := -O3 -mcpu=cortex-a9 -mtune=cortex-a9 -marm -mfpu=neon -mvectorize-with-neon-quad -munaligned-access -ffast-math -floop-block -floop-interchange -floop-nest-optimize -floop-parallelize-all -floop-strip-mine
obj-$(CONFIG_ION) +=	ion.o ion_heap.o ion_system_heap.o ion_carveout_heap.o
obj-$(CONFIG_ION_IOMMU)	+= ion_iommu_heap.o
obj-$(CONFIG_ION_TEGRA) += tegra/
