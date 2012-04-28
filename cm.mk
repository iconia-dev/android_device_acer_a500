# Release name
PRODUCT_RELEASE_NAME := A500

# Inherit some common CM stuff.
$(call inherit-product, vendor/cm/config/common_full_tablet_wifionly.mk)

# Inherit device configuration
$(call inherit-product, device/acer/a500/a500.mk)

## Device identifier. This must come after all inclusions
PRODUCT_DEVICE       := a500
PRODUCT_NAME         := cm_a500
PRODUCT_BRAND        := Acer
PRODUCT_MODEL        := A500
PRODUCT_MANUFACTURER := Acer

#Set build fingerprint / ID / Product Name ect.
PRODUCT_BUILD_PROP_OVERRIDES += \
    PRODUCT_NAME=picasso \
    BUILD_FINGERPRINT="acer/a500_ww_gen1/picasso:4.0.3/IML74K/1329824557:user/release-keys" \
    PRIVATE_BUILD_DESC="a500_ww_gen1-user 4.0.3 IML74K 1329824557 release-keys"

