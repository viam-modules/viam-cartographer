// Package cartofacade contains the api to call into CGO
package cartofacade

import (
	"context"
	"errors"
	"sync"
	"time"
)

// Initialize calls into the cartofacade C code.
func (cf *CartoFacade) Initialize(ctx context.Context, timeout time.Duration, activeBackgroundWorkers *sync.WaitGroup) error {
	cf.start(ctx, activeBackgroundWorkers)

	requestParams := map[RequestParamType]interface{}{}
	untyped, err := cf.request(ctx, initialize, requestParams, timeout)
	if err != nil {
		return err
	}

	carto, ok := untyped.(Carto)
	if !ok {
		return errors.New("unable to cast response from cartofacade to a carto struct")
	}

	cf.carto = &carto

	return nil
}

// Start calls into the cartofacade C code.
func (cf *CartoFacade) Start(ctx context.Context, timeout time.Duration) error {
	requestParams := map[RequestParamType]interface{}{}
	_, err := cf.request(ctx, start, requestParams, timeout)
	if err != nil {
		return err
	}

	return nil
}

// Stop calls into the cartofacade C code.
func (cf *CartoFacade) Stop(ctx context.Context, timeout time.Duration) error {
	requestParams := map[RequestParamType]interface{}{}
	_, err := cf.request(ctx, stop, requestParams, timeout)
	if err != nil {
		return err
	}

	return nil
}

// Terminate calls into the cartofacade C code.
func (cf *CartoFacade) Terminate(ctx context.Context, timeout time.Duration) error {
	requestParams := map[RequestParamType]interface{}{}
	_, err := cf.request(ctx, terminate, requestParams, timeout)
	if err != nil {
		return err
	}

	return nil
}

// AddSensorReading calls into the cartofacade C code.
func (cf *CartoFacade) AddSensorReading(
	ctx context.Context,
	timeout time.Duration,
	sensorName string,
	currentReading []byte,
	readingTimestamp time.Time,
) error {
	requestParams := map[RequestParamType]interface{}{
		sensor:    sensorName,
		reading:   currentReading,
		timestamp: readingTimestamp,
	}

	_, err := cf.request(ctx, addSensorReading, requestParams, timeout)
	if err != nil {
		return err
	}

	return nil
}

// GetPosition calls into the cartofacade C code.
func (cf *CartoFacade) GetPosition(ctx context.Context, timeout time.Duration) (GetPosition, error) {
	requestParams := map[RequestParamType]interface{}{}
	untyped, err := cf.request(ctx, position, requestParams, timeout)
	if err != nil {
		return GetPosition{}, err
	}

	pos, ok := untyped.(GetPosition)
	if !ok {
		return GetPosition{}, errors.New("unable to cast response from cartofacade to a position info struct")
	}

	return pos, nil
}

// GetInternalState calls into the cartofacade C code.
func (cf *CartoFacade) GetInternalState(ctx context.Context, timeout time.Duration) ([]byte, error) {
	requestParams := map[RequestParamType]interface{}{}
	untyped, err := cf.request(ctx, internalState, requestParams, timeout)
	if err != nil {
		return []byte{}, err
	}

	internalState, ok := untyped.([]byte)
	if !ok {
		return []byte{}, errors.New("unable to cast response from cartofacade to a byte slice")
	}

	return internalState, nil
}

// GetPointCloudMap calls into the cartofacade C code.
func (cf *CartoFacade) GetPointCloudMap(ctx context.Context, timeout time.Duration) ([]byte, error) {
	requestParams := map[RequestParamType]interface{}{}
	untyped, err := cf.request(ctx, pointCloudMap, requestParams, timeout)
	if err != nil {
		return []byte{}, err
	}

	pointCloud, ok := untyped.([]byte)
	if !ok {
		return []byte{}, errors.New("unable to cast response from cartofacade to a byte slice")
	}

	return pointCloud, nil
}
