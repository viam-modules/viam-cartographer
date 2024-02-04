// Package viamcartographer_test tests the functions that require injected components (such as robot and camera)
// in order to be run. It utilizes the internal package located in testhelper.go to access
// certain exported functions which we do not want to make available to the user. It also runs integration tests
// that test the interaction with the core C++ viam-cartographer code and the Golang implementation of the
// cartographer slam service.
package viamcartographer_test

import (
	"os"
	"path/filepath"
	"testing"
	"time"

	"go.viam.com/rdk/logging"
	"go.viam.com/test"

	viamcartographer "github.com/viamrobotics/viam-cartographer"
	"github.com/viamrobotics/viam-cartographer/cartofacade"
	"github.com/viamrobotics/viam-cartographer/testhelper"
)

// saveInternalState saves cartographer's internal state in the data directory.
func saveInternalState(t *testing.T, internalState []byte, dataDir string) string {
	timeStamp := time.Now().UTC()
	internalStateDir := filepath.Join(dataDir, "internal_state")
	err := os.Mkdir(internalStateDir, 0o755)
	test.That(t, err, test.ShouldBeNil)

	filename := filepath.Join(internalStateDir, "map_data_"+timeStamp.UTC().Format(testhelper.SlamTimeFormat)+".pbstream")
	err = os.WriteFile(filename, internalState, 0o644)
	test.That(t, err, test.ShouldBeNil)

	return filename
}

// TestIntegrationCartographer provides end-to-end testing of viam-cartographer using a combination of live vs. replay cameras
// and imu enabled mode.
func TestIntegrationCartographer(t *testing.T) {
	logger := logging.NewTestLogger(t)

	cases := []struct {
		description     string
		online          bool
		imuEnabled      bool
		odometerEnabled bool
		mode            cartofacade.SlamMode
		subAlgo         viamcartographer.SubAlgo
	}{
		// Online run with lidar only
		{
			description:     "online sensor mapping mode 2D",
			online:          true,
			imuEnabled:      false,
			odometerEnabled: false,
			mode:            cartofacade.MappingMode,
			subAlgo:         viamcartographer.Dim2d,
		},
		{
			description:     "online sensor localizing mode 2D",
			online:          true,
			imuEnabled:      false,
			odometerEnabled: false,
			mode:            cartofacade.LocalizingMode,
			subAlgo:         viamcartographer.Dim2d,
		},
		{
			description:     "online sensor updating mode 2D",
			online:          true,
			imuEnabled:      false,
			odometerEnabled: false,
			mode:            cartofacade.UpdatingMode,
			subAlgo:         viamcartographer.Dim2d,
		},
		// Offline run with lidar only
		{
			description:     "offline sensor mapping mode 2D",
			online:          false,
			imuEnabled:      false,
			odometerEnabled: false,
			mode:            cartofacade.MappingMode,
			subAlgo:         viamcartographer.Dim2d,
		},
		{
			description:     "offline sensor updating mode 2D",
			online:          false,
			imuEnabled:      false,
			odometerEnabled: false,
			mode:            cartofacade.UpdatingMode,
			subAlgo:         viamcartographer.Dim2d,
		},
		// Online run with lidar + imu
		{
			description:     "online with imu sensor mapping mode 2D",
			online:          true,
			imuEnabled:      true,
			odometerEnabled: false,
			mode:            cartofacade.MappingMode,
			subAlgo:         viamcartographer.Dim2d,
		},
		{
			description:     "online with imu sensor localizing mode 2D",
			online:          true,
			imuEnabled:      true,
			odometerEnabled: false,
			mode:            cartofacade.LocalizingMode,
			subAlgo:         viamcartographer.Dim2d,
		},
		{
			description:     "online with imu sensor updating mode 2D",
			online:          true,
			imuEnabled:      true,
			odometerEnabled: false,
			mode:            cartofacade.UpdatingMode,
			subAlgo:         viamcartographer.Dim2d,
		},
		// // Online run with lidar + odometer
		// {
		// 	description:     "online with odometer sensor mapping mode 2D",
		// 	online:          true,
		// 	imuEnabled:      false,
		// 	odometerEnabled: true,
		// 	mode:            cartofacade.MappingMode,
		// 	subAlgo:         viamcartographer.Dim2d,
		// },
		// {
		// 	description:     "online with odometer sensor localizing mode 2D",
		// 	online:          true,
		// 	imuEnabled:      false,
		// 	odometerEnabled: true,
		// 	mode:            cartofacade.LocalizingMode,
		// 	subAlgo:         viamcartographer.Dim2d,
		// },
		// {
		// 	description:     "online with odometer sensor updating mode 2D",
		// 	online:          true,
		// 	imuEnabled:      false,
		// 	odometerEnabled: true,
		// 	mode:            cartofacade.UpdatingMode,
		// 	subAlgo:         viamcartographer.Dim2d,
		// },
	}

	// Loop over defined test cases, resetting the directories between slam sessions
	for _, tt := range cases {
		t.Run(tt.description, func(t *testing.T) {
			// 1. Run cartographer in mapping mode
			enableMapping := true

			// Prep first run directory
			dataDirectory1, err := os.MkdirTemp("", "*")
			test.That(t, err, test.ShouldBeNil)
			defer func() {
				err := os.RemoveAll(dataDirectory1)
				test.That(t, err, test.ShouldBeNil)
			}()

			// Run mapping test
			internalState := testhelper.IntegrationCartographer(
				t, "", tt.subAlgo, logger, tt.online,
				tt.imuEnabled, tt.odometerEnabled, enableMapping, cartofacade.MappingMode,
			)

			// 2. Return if we're only testing mapping mode, as we are done with
			// mapping at this point
			if tt.mode == cartofacade.MappingMode {
				return
			}

			// 3. Run cartographer either in localizing or updating mode
			if tt.mode == cartofacade.LocalizingMode {
				enableMapping = false
			}

			// Prep second run directory
			dataDirectory2, err := os.MkdirTemp("", "*")
			test.That(t, err, test.ShouldBeNil)
			defer func() {
				err := os.RemoveAll(dataDirectory2)
				test.That(t, err, test.ShouldBeNil)
			}()

			// Save internal state
			existingMap := saveInternalState(t, internalState, dataDirectory2)
			test.That(t, existingMap, test.ShouldNotEqual, "")

			// Run follow-up updating or localizing test
			testhelper.IntegrationCartographer(
				t, existingMap, tt.subAlgo, logger, tt.online,
				tt.imuEnabled, tt.odometerEnabled, enableMapping, tt.mode,
			)
		})
	}
}
