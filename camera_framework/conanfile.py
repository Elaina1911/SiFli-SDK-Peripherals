from conan import ConanFile

class Ov2640Recipe(ConanFile):
    name = "ov2640"
    version = "0.0.2"

    license = "Apache-2.0"
    user = "sifli"
    author = "sifli"
    url = "https://github.com/OpenSiFli/SiFli-SDK-Peripherals"
    homepage = "https://packages.sifli.com/zh/packages/sifli"
    description = "SiFli SDK Peripherals - OV2640 Camera Module"
    topics = ("camera", "ov2640", "peripherals")

    support_sdk_version = "2.4"

    # Sources are located in the same place as this recipe, copy them to the recipe
    exports_sources = "*"

    python_requires = "sf-pkg-base/[^1.0]@sifli"
    python_requires_extend = "sf-pkg-base.SourceOnlyBase"

    def requirements(self):
        # add your package dependencies here, for example:
        # self.requires("fmt/8.1.1")
        pass
