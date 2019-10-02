#!/usr/bin/env python

from __future__ import division

"""
Class implementation of quantization algorithm.
"""

import os
import math
import yaml
import numpy as np
from decimal import Decimal, getcontext
from scipy.linalg import sqrtm
getcontext().prec = 500

class Quantizer:
    """
    Class implementation of measurement and state quantization algorithm.
    Compresses floating point data for transmission in severely bandwidth-
    limited applications.

    Exposes interfaces for transmitting measurements and states, as well as
    a general purpose string of data.

    Parameters
    ----------
    quantizer_fxn
        string name of desired quantization function, e.g. uniform, x^2 binning, etc.
        Fxns: uniform, piecewise x2 (f(x)=ax^2 + bx + c)
    config_path
        path to yaml config file specifying quantization function parameters and
        measurement and state encoding parameters.
    """
    def __init__(self,quantizer_fxn='uniform',config_path='../config/quantization_config.yaml'):
        
        # ensure config path is absolute
        config_path = os.path.abspath(config_path)
        # load config
        with open(config_path,'r') as f:
            self.cfg = yaml.load(f,Loader=yaml.SafeLoader)

        # check that quant fxn is allowable type
        try:
            assert(quantizer_fxn in self.cfg['quant_types'])
            self.quantizer_fxn = quantizer_fxn
        except AssertionError:
            print('Passed quantization function is not implemented type!')
            print('Available types: {}'.format(self.cfg['quant_types']))
            print('Falling back to uniform quantization.')

        # number of available bits for message
        self.available_bits = self.cfg['available_bits']

    def meas2quant(self,measurement,type_=None,measurement_range=None,measurement_resolution=None,bits_flag=True,config_flag=False):
        """
        Convert measurement into quantized representation using parameters
        from config or passed.

        Parameters
        ----------
        measurement
            measurement data to be transmitted
        type_ : [default - None]
            measurement type for parameter lookup in quantization config
        measurement_range : [default - None]
            list of measurement element ranges (length=# of measurement elements)
            Note: not used if type_ is passed
        measurement_resolution : [default - None]
            list of measurement element resolutions (length=# of measurement elements)
            Note: not used if type_ is passed
        bits_flag: [default - True]
            return the bitstring of the quantized measurement
        config_flag: [default - False]
            return the configuration number before conversion to bitstring
        """
        # retrieve measurement parameters
        if type_ in self.cfg['meas_types']:
            # measurement_range = self.cfg['meas_types'][type_]['range']
            # measurement_resolution = self.cfg['meas_types'][type_]['resolution']
            element_types = self.cfg['meas_types'][type_]
            if self.quantizer_fxn == 'uniform':
                measurement_range = [self.cfg[element]['range'] for element in element_types]
                measurement_resolution = [self.cfg[element]['resolution'] for element in element_types]
            else:
                measurement_range = [self.cfg[element]['range'] for element in element_types]
                measurement_center = [self.cfg[element]['center'] for element in element_types]
                measurement_num_bins = [self.cfg[element]['num_bins'] for element in element_types]

        # compute bins, with method depending on quantizer fxn
        if self.quantizer_fxn == 'uniform':
            # compute number of bins --> note that instead of dividing by the resolution,
            # we use the following method to avoid floating point error
            num_bins = []
            for i in range(0,len(measurement_range)):
                num_bins_element = Decimal(measurement_range[i][1]-measurement_range[i][0])*(Decimal(10**(-1*np.log10(measurement_resolution[i]))))
                num_bins.append(num_bins_element)
            quantization_params = {'resolution': measurement_resolution, 'range': measurement_range, 'num_bins': num_bins}
        elif self.quantizer_fxn == 'x2':
            # for adaptive binning, we have the number of bins already, and
            # need to compute the scaling on the function using the range and
            # number of bins
            quantization_params = {'range': measurement_range, 'center': measurement_center, 'num_bins': measurement_num_bins}

        # compute bin for each element
        bin_list = self.vals2bins(measurement,quantization_params)

        # compute config number
        config_num = self.bin2config(bin_list,quantization_params['num_bins'])

        # convert to binary
        bits = bin(int(config_num))

        return_vals = []
        if bits_flag: return_vals.append(bits)
        if config_flag: return_vals.append(config_num)
        
        return return_vals

    def quant2meas(self,bitstring,num_els,type_=None,measurement_range=None,measurement_resolution=None,config_num=None):
        """
        Convert quantized measurement to measurement using range and resolution.

        Parameters
        ----------
        bitstring
            bits representing configuration number
        num_els
            expected number of elements for message
        type_
            message type (e.g. measurement type)
        param_struct
            structure with quantization parameters
        config_num : [default None]
            int version of configuration number to use instead of bitstring.
            Note: this value is used instead of bitstring if both are passed.
        """
        # convert from bits to configuration number
        if config_num is None:
            config_num = int(bitstring,base=2)
        config_num = Decimal(config_num)

        # retrieve measurement parameters
        if type_ in self.cfg['meas_types']:
            element_types = self.cfg['meas_types'][type_]
            if self.quantizer_fxn == 'uniform':
                measurement_range = [self.cfg[element]['range'] for element in element_types]
                measurement_resolution = [self.cfg[element]['resolution'] for element in element_types]
            else:
                measurement_range = [self.cfg[element]['range'] for element in element_types]
                measurement_center = [self.cfg[element]['center'] for element in element_types]
                measurement_num_bins = [self.cfg[element]['num_bins'] for element in element_types]

        # compute bins, with method depending on quantizer fxn
        if self.quantizer_fxn == 'uniform':
            # compute number of bins --> note that instead of dividing by the resolution,
            # we use the following method to avoid floating point error
            num_bins = []
            for i in range(0,len(measurement_range)):
                num_bins_element = Decimal(measurement_range[i][1]-measurement_range[i][0])*(Decimal(10**(-1*np.log10(measurement_resolution[i]))))
                num_bins.append(num_bins_element)
            quantization_params = {'resolution': measurement_resolution, 'range': measurement_range, 'num_bins': num_bins}
        elif self.quantizer_fxn == 'x2':
            # for adaptive binning, we have the number of bins already, and
            # need to compute the scaling on the function using the range and
            # number of bins
            quantization_params = {'range': measurement_range, 'center': measurement_center, 'num_bins': measurement_num_bins}

        # compute bin numbers
        bin_num_list = self.config2bins(config_num,num_els,quantization_params['num_bins'])

        # compute element values from bin numbers
        measurement_list = self.bins2vals(bin_num_list,quantization_params)

        return measurement_list

    def state2quant(self,mean_vec,cov,element_types,mean_range=None,mean_resolution=None,diag_range=None,
        diag_resolution=None,offdiag_range=None,offdiag_resolution=None,bits_flag=True,config_flag=False,
        mean_num_bins=None,diag_num_bins=None,offdiag_num_bins=None,diag_only=False):
        """
        Convert state estimate to quantized representation using config values or 
        passed parameter struct.

        Parameters
        ----------
        mean_vec
            mean vector of state estimate -- n x 1 numpy array
        cov
            full covariance matrix of estimate -- n x n numpy array
        element types
            n element list of estimate state element types, e.g. ['position','velocity']
        mean_range : [default None]
            range of values for mean vector
        mean_resolution : [default None]

        diag_range : [default None]

        diag_resolution : [default None]

        offdiag_range : [default None]

        offdiag_resolution : [default None]

        bits_flag : [default True]
            return bitstring of quantized state estimate
        config_flag : [default False]
            return integer config number of quantized state estimate
        diag_only : [default False]
            only quantize the state mean vector and covariance diagonals (for use with diagonalization techniques)
        """
        if self.quantizer_fxn == 'uniform':
            # get ranges and resolutions from config
            mean_range = [self.cfg[element]['range'] for element in element_types]
            mean_resolution = [self.cfg[element]['resolution'] for element in element_types]
            
            diag_range = [self.cfg[element]['variance_range'] for element in element_types]
            diag_resolution = [self.cfg[element]['variance_resolution'] for element in element_types]

            offdiag_range = [self.cfg['covar_offdiag_range'] for i in range(0,int(0.5*len(element_types)*(len(element_types)-1)))]
            offdiag_resolution = [self.cfg['covar_offdiag_resolution'] for i in range(0,int(0.5*len(element_types)*(len(element_types)-1)))]
        else:
            mean_range = [self.cfg[element]['range'] for element in element_types]
            diag_range = [self.cfg[element]['variance_range'] for element in element_types]
            offdiag_range = [self.cfg['covar_offdiag_range'] for i in range(0,int(0.5*len(element_types)*(len(element_types)-1)))]

            mean_center = [self.cfg[element]['center'] for element in element_types]
            diag_center = [self.cfg[element]['variance_center'] for element in element_types]
            offdiag_center = [self.cfg['covar_offdiag_center'] for i in range(0,int(0.5*len(element_types)*(len(element_types)-1)))]

            if mean_num_bins is None:
                mean_num_bins = [self.cfg[element]['num_bins'] for element in element_types]
            if diag_num_bins is None:
                diag_num_bins = [self.cfg[element]['variance_num_bins'] for element in element_types]
            if offdiag_num_bins is None:
                offdiag_num_bins = [self.cfg['covar_offdiag_num_bins'] for i in range(0,int(0.5*len(element_types)*(len(element_types)-1)))]

        if self.quantizer_fxn == 'uniform':
            # first compute number of bins
            mean_num_bins = []
            for i in range(0,len(mean_range)):
                mean_bins_element = int((mean_range[i][1]-mean_range[i][0])*(10**(-1*np.log10(mean_resolution[i]))))
                mean_num_bins.append(mean_bins_element)

            diag_num_bins = []
            for i in range(0,len(diag_range)):
                diag_bins_element = int((diag_range[i][1]-diag_range[i][0])*(10**(-1*np.log10(diag_resolution[i]))))
                diag_num_bins.append(diag_bins_element)

            offdiag_num_bins = []
            for i in range(0,len(offdiag_range)):
                offdiag_bins_element = int((offdiag_range[i][1]-offdiag_range[i][0])*(10**(-1*np.log10(offdiag_resolution[i]))))
                offdiag_num_bins.append(offdiag_bins_element)

            element_resolution = mean_resolution + diag_resolution #+ offdiag_resolution
            element_range = mean_range + diag_range #+ offdiag_range
            num_bins = mean_num_bins + diag_num_bins #+ offdiag_num_bins

            if not diag_only:
                element_resolution += offdiag_resolution
                element_range += offdiag_range
                num_bins += offdiag_num_bins

            quantization_params = {'resolution': element_resolution, 'range': element_range, 'num_bins': num_bins}

        elif self.quantizer_fxn == 'x2':
            # combine the parameters from mean, diagonal, and off-diagonal
            element_range = mean_range + diag_range #+ offdiag_range
            element_center = mean_center + diag_center #+ offdiag_center
            element_num_bins = mean_num_bins + diag_num_bins #+ offdiag_num_bins

            if not diag_only:
                element_range += offdiag_range
                element_center += offdiag_center
                element_num_bins += offdiag_num_bins

            quantization_params = {'range': element_range, 'center': element_center, 'num_bins': element_num_bins}

        # extract diagonal cov elements
        diag_els = np.diag(cov)
        # extract off-diagonal, upper-triangular elements
        offdiag_els = np.extract(np.triu(1-np.eye(cov.shape[0])),cov)

        # combine both elements and max number of bin lists of mean, diagonal, and off-diagonal
        mean_vec = np.squeeze(mean_vec)
        elements = np.concatenate((mean_vec,diag_els))
        if not diag_only:
            elements = np.concatenate((elements,offdiag_els))

        bin_list = self.vals2bins(elements,quantization_params)

        config_num = self.bin2config(bin_list,quantization_params['num_bins'])

        # convert to bitstring
        bits = bin(int(config_num))

        return_vals = []
        if bits_flag: return_vals.append(bits)
        if config_flag: return_vals.append(config_num)
        
        return return_vals

    def quant2state(self,bitstring,num_els,element_types,mean_range=None,mean_resolution=None,
                    diag_range=None,diag_resolution=None,offdiag_range=None,offdiag_resolution=None,config_num=None,
                    mean_num_bins=None,diag_num_bins=None,offdiag_num_bins=None,diag_only=False):
        """
        Convert quantized state to state using config values or passed ranges and resolutions.

        Parameters
        ----------
        bitstring
            bitstring of quantized measurements
        num_els
            number of elements to convert to values
            NOTE: should be equal to the n+((n^2+n)/2), or 2n if diag_only
        element_types
            state element types
        mean_range : [default None]
            range of values for mean vector
        mean_resolution : [default None]

        diag_range : [default None]

        diag_resolution : [default None]

        offdiag_range : [default None]

        offdiag_resolution : [default None]

        config_num : [default False]
            use integer config number instead of bistring
        """
        # convert from bits to configuration number
        if config_num is None:
            config_num = int(bitstring,base=2)
        config_num = Decimal(value=config_num)

        if self.quantizer_fxn == 'uniform':
            # get ranges and resolutions from config
            mean_range = [self.cfg[element]['range'] for element in element_types]
            mean_resolution = [self.cfg[element]['resolution'] for element in element_types]
            
            diag_range = [self.cfg[element]['variance_range'] for element in element_types]
            diag_resolution = [self.cfg[element]['variance_resolution'] for element in element_types]

            offdiag_range = [self.cfg['covar_offdiag_range'] for i in range(0,int(0.5*len(element_types)*(len(element_types)-1)))]
            offdiag_resolution = [self.cfg['covar_offdiag_resolution'] for i in range(0,int(0.5*len(element_types)*(len(element_types)-1)))]
        else:
            mean_range = [self.cfg[element]['range'] for element in element_types]
            diag_range = [self.cfg[element]['variance_range'] for element in element_types]
            offdiag_range = [self.cfg['covar_offdiag_range'] for i in range(0,int(0.5*len(element_types)*(len(element_types)-1)))]

            mean_center = [self.cfg[element]['center'] for element in element_types]
            diag_center = [self.cfg[element]['variance_center'] for element in element_types]
            offdiag_center = [self.cfg['covar_offdiag_center'] for i in range(0,int(0.5*len(element_types)*(len(element_types)-1)))]

            if mean_num_bins is None:
                mean_num_bins = [self.cfg[element]['num_bins'] for element in element_types]
            if diag_num_bins is None:
                diag_num_bins = [self.cfg[element]['variance_num_bins'] for element in element_types]
            if offdiag_num_bins is None:
                offdiag_num_bins = [self.cfg['covar_offdiag_num_bins'] for i in range(0,int(0.5*len(element_types)*(len(element_types)-1)))]

        if self.quantizer_fxn == 'uniform':
            # first compute number of bins
            mean_num_bins = []
            for i in range(0,len(mean_range)):
                mean_bins_element = int((mean_range[i][1]-mean_range[i][0])*(10**(-1*np.log10(mean_resolution[i]))))
                mean_num_bins.append(mean_bins_element)

            diag_num_bins = []
            for i in range(0,len(diag_range)):
                diag_bins_element = int((diag_range[i][1]-diag_range[i][0])*(10**(-1*np.log10(diag_resolution[i]))))
                diag_num_bins.append(diag_bins_element)

            offdiag_num_bins = []
            for i in range(0,len(offdiag_range)):
                offdiag_bins_element = int((offdiag_range[i][1]-offdiag_range[i][0])*(10**(-1*np.log10(offdiag_resolution[i]))))
                offdiag_num_bins.append(offdiag_bins_element)

            element_resolution = mean_resolution + diag_resolution #+ offdiag_resolution
            element_range = mean_range + diag_range #+ offdiag_range
            num_bins = mean_num_bins + diag_num_bins #+ offdiag_num_bins

            if not diag_only:
                element_resolution += offdiag_resolution
                element_range += offdiag_range
                num_bins += offdiag_num_bins

            quantization_params = {'resolution': element_resolution, 'range': element_range, 'num_bins': num_bins}

        elif self.quantizer_fxn == 'x2':
            # for adaptive binning, we have the number of bins already, and
            # need to compute the scaling on the function using the range and
            # number of bins
            # mean_scale_list = [mean_range[i][1]/((0.5*mean_num_bins[i]+mean_center[i])**3) for i in range(0,len(mean_range))]
            # diag_scale_list = [diag_range[i][1]/((0.5*diag_num_bins[i]+diag_center[i])**3) for i in range(0,len(diag_range))]
            # offdiag_scale_list = [offdiag_range[i][1]/((0.5*offdiag_num_bins[i]+offdiag_center[i])**3) for i in range(0,len(offdiag_range))]

            # element_scale_list = mean_scale_list + diag_scale_list + offdiag_scale_list
            element_range = mean_range + diag_range #+ offdiag_range
            element_center = mean_center + diag_center #+ offdiag_center
            element_num_bins = mean_num_bins + diag_num_bins #+ offdiag_num_bins

            if not diag_only:
                element_range += offdiag_range
                element_center += offdiag_center
                element_num_bins += offdiag_num_bins

            quantization_params = {'range': element_range, 'center': element_center, 'num_bins': element_num_bins}

        # compute bin numbers
        bin_num_list = self.config2bins(config_num, num_els, quantization_params['num_bins'])

        # compute values
        values = self.bins2vals(bin_num_list,quantization_params)

        # recover mean and covariance
        mean_vec = np.array(values[:len(element_types)])
        mean_vec = np.atleast_2d(mean_vec)

        diag_vec = values[len(element_types):2*len(element_types)]
        cov = np.diag(diag_vec)

        if not diag_only:
            offdiag_vec = values[2*len(element_types):]
            off_diag_els_u = np.triu(1-np.eye(cov.shape[0]))
            off_diag_els_u = off_diag_els_u.astype(bool)
            off_diag_els_l = np.tril(1-np.eye(cov.shape[0]))
            off_diag_els_l = off_diag_els_l.astype(bool)
            cov[off_diag_els_u] = offdiag_vec
            cov[off_diag_els_l] = offdiag_vec

        return mean_vec, cov

    def vals2bins(self, vals, quantization_params, range_warn=False):
        """
        Compute the bin numbers for a quantized version of vals at given range and resolution.
        """
        bin_num_list = []
        for i,el in enumerate(vals):

            # make sure element is within range
            if el < quantization_params['range'][i][0]:
                el = quantization_params['range'][i][0]
                if range_warn:
                    print('Warning: Element {} is outside allowable range, setting to min.'.format(i))
            elif el > quantization_params['range'][i][1]:
                el = quantization_params['range'][i][1]
                if range_warn:
                    print('Warning: Element {} is outside allowable range, setting to max.'.format(i))

            if self.quantizer_fxn == 'uniform':
                # determine rounding method (depends on if resolution is smaller than 1)
                if quantization_params['resolution'][i] < 1:
                    # shift range to 0, then round to resolution
                    rounded = round(el-quantization_params['range'][i][0],int(abs(np.log10(quantization_params['resolution'][i]))))
                    bin_num = rounded * (10**(-1*np.log10(quantization_params['resolution'][i])))
                    bin_num_list.append((int(bin_num)))
            elif self.quantizer_fxn == 'x2':
                # compute bin values
                lower_bound = quantization_params['range'][i][0]
                upper_bound = quantization_params['range'][i][1]
                num_bins = quantization_params['num_bins'][i]
                # scale = quantization_params['scale'][i]
                center = quantization_params['center'][i]
                # bins = scale*np.arange(center-0.5*num_bins,center+0.5*num_bins+1)**3
                # binning fxn: params found by solving linear system using config params
                # composed of two functions, upper and lower, to all constraints can be satisfied
                a_l = -4*(center - lower_bound) / (num_bins**2)
                b_l = 4*(center - lower_bound) / num_bins
                c_l = lower_bound

                a_u = -4*(center - upper_bound) / (num_bins**2)
                b_u = 4*(center - upper_bound) / num_bins
                c_u = upper_bound

                binning_fxn = lambda a,b,c,x: a*x**2 + b*x + c
                binning_fxn_derivative = lambda a,b,x: 2*a*x + b

                # compute bin values using binning function
                bins = np.arange(0,num_bins+1,1)

                bin_vals_lower = binning_fxn(a_l,b_l,c_l,bins)
                bin_vals_derivative_lower = binning_fxn_derivative(a_l,b_l,bins)
                bin_vals_upper = binning_fxn(a_u,b_u,c_u,bins)
                bin_vals_derivative_upper = binning_fxn_derivative(a_u,b_u,bins)
                
                # join functions at center
                bin_vals = np.concatenate((bin_vals_lower[:int(0.5*num_bins)], bin_vals_upper[int(0.5*num_bins):]))

                # find which bins values belong to
                bin_idx = np.digitize(el,bin_vals)
                bin_num_list.append(int(bin_idx))
        
        return bin_num_list

    def bin2config(self, bin_num_list, num_bins):
        """
        Compute configuration number from list of element bin numbers and total
        number of possible bins for each element.
        """
        config_num = Decimal(bin_num_list[0])

        for i in range(1,len(bin_num_list)):
            prod_val = Decimal(1)
            for j in range(1,i+1):
                prod_val *= Decimal(num_bins[j-1])
            config_num += prod_val*(Decimal(bin_num_list[i])-Decimal(1))

        return config_num

    def config2bins(self, config_num, num_els, num_bins):
        """
        Convert from received configuration number to bin numbers.
        """
        bin_num_list = []
        for i in range(1,num_els+1):
            if i != num_els:
                bin_prod = Decimal(1)
                for j in range(0,len(num_bins)-i):
                    bin_prod *= Decimal(num_bins[j])
                bin_num = Decimal(math.floor(config_num/bin_prod)) + Decimal(1)
                config_num = config_num - bin_prod*(Decimal(bin_num)-Decimal(1))
            else:
                bin_num = config_num
            bin_num_list.append(bin_num)

        return bin_num_list

    def bins2vals(self, bins, quantization_params):
        """
        Convert from bin numbers to element values.

        Parameters
        ----------
        bins
            list of bin numbers for each element of encoded message
        val_resolution
            resolution of elements
        val_range
            range of element values
        """
        if self.quantizer_fxn == 'uniform':
            # reverse resolution and range
            quantization_params['resolution'].reverse()
            quantization_params['range'].reverse()
        else:
            quantization_params['range'].reverse()
            quantization_params['num_bins'].reverse()
            quantization_params['center'].reverse()

        elements = []
        for i in range(0,len(bins)):
            if self.quantizer_fxn == 'uniform':
                el_val = bins[i]*Decimal(quantization_params['resolution'][i]) + Decimal(quantization_params['range'][i][0])
            elif self.quantizer_fxn == 'x2':
                # compute bin values
                # el_val = Decimal(quantization_params['scale'][i])*(bins[i]-quantization_params['center'][i]-Decimal(0.5*quantization_params['num_bins'][i]))**3
                # el_val = 
                lower_bound = quantization_params['range'][i][0]
                upper_bound = quantization_params['range'][i][1]
                num_bins = quantization_params['num_bins'][i]
                # scale = quantization_params['scale'][i]
                center = quantization_params['center'][i]
                # bins = scale*np.arange(center-0.5*num_bins,center+0.5*num_bins+1)**3
                # binning fxn: params found by solving linear system using config params
                # composed of two functions, upper and lower, to all constraints can be satisfied
                a_l = -4*(center - lower_bound) / (num_bins**2)
                b_l = 4*(center - lower_bound) / num_bins
                c_l = lower_bound

                a_u = -4*(center - upper_bound) / (num_bins**2)
                b_u = 4*(center - upper_bound) / num_bins
                c_u = upper_bound

                binning_fxn = lambda a,b,c,x: Decimal(a)*Decimal(x)**2 + Decimal(b)*Decimal(x) + Decimal(c)

                # determine if upper or lower function needs to be used
                if bins[i] > 0.5*num_bins: el_val = binning_fxn(a_u,b_u,c_u,bins[i])
                elif bins[i] < 0.5*num_bins: el_val = binning_fxn(a_l,b_l,c_l,bins[i])
                else: el_val = center

            elements.append(float(el_val))

        elements.reverse()

        return elements

def covar_diagonalize(P,method='eig'):
    """
    Diagonalization of covariance matrices to reduce DDF data transfer.
    See Forsling et. al. FUSION 2019 for methods implemented and more info.
    """
    if method == 'eig':
        # compute diagonal covariance
        D = np.diag(np.diag(P))
        # compute correlation matrix
        Q = np.dot(np.linalg.inv(sqrtm(D)),np.dot(P,np.linalg.inv(sqrtm(D))))
        # find largest eigenvalue of Q
        lamb = max(np.linalg.eig(Q)[0])
        # inflate diagonalized covariance
        Dc = lamb*D

    return Dc

if __name__ == "__main__":
    q = Quantizer(quantizer_fxn='x2')

    meas = [100.289, 0.439012, 1.209123]
    # meas = [30.2128979]
    meas[1] *= 180/np.pi
    meas[2] *= 180/np.pi
    print(meas)
    meas[1] *= np.pi/180
    meas[2] *= np.pi/180
    meas_type = 'usbl'

    bits = q.meas2quant(meas,type_=meas_type)
    print(bits[0],len(bits[0]))
    meas_decoded = q.quant2meas(bits[0],len(meas),type_=meas_type)
    meas_decoded[1] *= 180/np.pi
    meas_decoded[2] *= 180/np.pi
    print(meas_decoded)

    print('---------------------------------')

    # state1_mean = [-0.89, 0.2345,0.9]
    # state1_cov = np.array([[1,0.1,-0.674],
    #                 [0.1,1.5,0.216],
    #                 [-0.674,0.216,1.75]])
    # element_types = ['position','velocity','angle']
    # element_types = ['position','position','position']
    
    # state1_mean = [103.10290923,5.1093]
    # state1_cov = np.array([[1003,0.030982],
    #                         [0.030982,25.21098]])
    # element_types = ['position','velocity']

    state1_mean = [-0.89, 0.2345,10.20912,1.302,-3.4311,-0.5922]
    state1_cov = np.array([ [1,0.1,-0.674,0.2198,-0.129,1.1901],
                            [0.1,10.5,0.291,0.2198,0.1,0.1],
                            [-0.674,0.291,20.10909,0.2198,-0.129,1.1901],
                            [0.2198,0.2198,0.2198,5.292,-0.129,0.2198],
                            [-0.129,0.1,-0.129,-0.129,50.309,1.1901],
                            [1.1901,0.1,0.2198,0.2198,1.1901,3.290]])
    element_types = ['position','velocity','position','velocity','position','velocity']
    # element_types = ['position','position','position','position','position','position']

    print(state1_mean)
    print(state1_cov)

    bits = q.state2quant(state1_mean,state1_cov,element_types)
    print(bits[0],len(bits[0]))
    num_els = int(len(element_types) + 0.5*len(element_types)*(len(element_types)+1))
    mean,cov = q.quant2state(bits[0],num_els,element_types)
    print(mean)
    print(cov)