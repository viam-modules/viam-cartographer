BUILD_CHANNEL?=local

appimage: build
	cd etc/packaging/appimages && BUILD_CHANNEL=${BUILD_CHANNEL} appimage-builder --recipe cartographer-module-`uname -m`.yml
	if [ "${RELEASE_TYPE}" = "stable" ]; then \
		cd etc/packaging/appimages; \
		BUILD_CHANNEL=stable appimage-builder --recipe cartographer-module-`uname -m`.yml; \
	fi
	mkdir -p etc/packaging/appimages/deploy/
	mv etc/packaging/appimages/*.AppImage* etc/packaging/appimages/deploy/
	chmod 755 etc/packaging/appimages/deploy/*.AppImage

# AppImage packaging targets run in canon docker
appimage-multiarch: appimage-amd64 appimage-arm64

appimage-amd64:
	canon --arch amd64 make appimage

appimage-arm64:
	canon --arch arm64 make appimage

appimage-deploy:
	gsutil -m -h "Cache-Control: no-cache" cp etc/packaging/appimages/deploy/* gs://packages.viam.com/apps/cartographer-module/

.PHONY: clean-appimage
clean-appimage:
	rm -rf etc/packaging/appimages/AppDir && rm -rf etc/packaging/appimages/appimage-build && rm -rf etc/packaging/appimages/deploy
