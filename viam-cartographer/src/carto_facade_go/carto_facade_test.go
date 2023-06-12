package carto_facade_test

import (
	"context"
	"testing"

	cartoFacade "github.com/viamrobotics/viam-cartographer/viam-cartographer/src/carto_facade_go"
	"go.viam.com/test"
)

func TestViamCartoCGoAPI(t *testing.T) {
	pvcl, err := cartoFacade.NewViamCartoLib(1, 1)

	t.Run("initialize viam_carto_lib", func(t *testing.T) {
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pvcl, test.ShouldNotBeNil)
	})

	vc, err := cartoFacade.NewViamCarto(*pvcl)
	t.Run("initialize viam_carto", func(t *testing.T) {
		test.That(t, err, test.ShouldBeNil)
		test.That(t, vc, test.ShouldNotBeNil)
	})

	t.Run("terminate viam_carto", func(t *testing.T) {
		err = vc.TerminateViamCarto(context.Background())
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("terminate viam_carto_lib", func(t *testing.T) {
		err = pvcl.TerminateViamCartoLib()
		test.That(t, err, test.ShouldBeNil)
	})
}

// func TestStart(t *testing.T) {
// 	vcl := cartoFacade.NewCViamCartoLibObj()
// 	vc := cartoFacade.NewCViamCartoObj(vcl)
// 	err := vc.Start(context.Background())

// 	test.That(t, err, test.ShouldBeNil)
// }

// func TestStop(t *testing.T) {
// 	vcl := cartoFacade.NewCViamCartoLibObj()
// 	vc := cartoFacade.NewCViamCartoObj(vcl)
// 	err := vc.Stop(context.Background())

// 	test.That(t, err, test.ShouldBeNil)
// }

// func TestAddSensorReading(t *testing.T) {
// 	vcl := cartoFacade.NewCViamCartoLibObj()
// 	vc := cartoFacade.NewCViamCartoObj(vcl)
// 	err := vc.AddSensorReading(context.Background())

// 	test.That(t, err, test.ShouldBeNil)
// }

// func TestGetPosition(t *testing.T) {
// 	vcl := cartoFacade.NewCViamCartoLibObj()
// 	vc := cartoFacade.NewCViamCartoObj(vcl)
// 	pos, name, err := vc.GetPosition(context.Background())

// 	test.That(t, err, test.ShouldBeNil)

// 	test.That(t, name, test.ShouldEqual, "")

// 	// Question: Actually getting 100.00000000000003 for pos.Point().X, is this error acceptable?
// 	test.That(t, pos.Point().X, test.ShouldAlmostEqual, 100, .001)
// 	test.That(t, pos.Point().Y, test.ShouldAlmostEqual, 200, .001)
// 	test.That(t, pos.Point().Z, test.ShouldAlmostEqual, 300, .001)

// 	// Question: These values get converted when put into NewPose, is this correct?
// 	test.That(t, pos.Orientation().OrientationVectorDegrees().OX, test.ShouldAlmostEqual, .455, .001)
// 	test.That(t, pos.Orientation().OrientationVectorDegrees().OY, test.ShouldAlmostEqual, .569, .001)
// 	test.That(t, pos.Orientation().OrientationVectorDegrees().OZ, test.ShouldAlmostEqual, .683, .001)
// }

// func TestGetPointCloudMap(t *testing.T) {
// 	vcl := cartoFacade.NewCViamCartoLibObj()
// 	vc := cartoFacade.NewCViamCartoObj(vcl)
// 	_, err := vc.GetPointCloudMap(context.Background())

// 	test.That(t, err, test.ShouldBeNil)
// }

// func TestGetInternalState(t *testing.T) {
// 	vcl := cartoFacade.NewCViamCartoLibObj()
// 	vc := cartoFacade.NewCViamCartoObj(vcl)
// 	_, err := vc.GetInternalState(context.Background())

// 	test.That(t, err, test.ShouldBeNil)
// }
